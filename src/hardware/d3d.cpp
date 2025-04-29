/*
 *  Copyright (C) 2002-2025  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */


#include "dosbox.h"
#include "inout.h"
#include "logging.h"
#include "mem.h"
#include "paging.h"
#include "setup.h"
#include "vga.h"
#include "dos_inc.h"      /* for Drives[] */
#include "control.h"
#include "sdlmain.h"
#include "../dos/drives.h"

#include <iomanip>
#include <sstream>
#include <vector>
#include <map>
using namespace std;

#include "SDL.h"
#include "SDL_syswm.h"

#if defined (WIN32)
#include <windows.h>
#include <Shlwapi.h>

#ifdef _MSC_VER
#pragma comment( lib, "Shlwapi" )
#endif

#else // *nix
#include <dlfcn.h>

#include <dirent.h>
#include <errno.h>

#endif

extern bool OpenGL_using(void);
extern void GFX_Stop(void);
extern void GFX_ResetScreen(void);
extern const char* RunningProgram;
extern bool dpi_aware_enable;
extern bool DOS_GetMemory_unmapped;

static float int_to_float(const uint32_t i)
{
    float f;
    uint32_t i_native = SDL_SwapLE32(i);
    SDL_memcpy(&f, &i_native, 4);
    return f;
}

#define SAFE_DELETE(p) { if(p) { delete p; p = NULL; } }

#define D3D_OK      1
#define D3D_FAIL    0

// Print debug messages
#define LOG_D3D 1

// PCI register access macros
#define PCI_READ(reg) d3d_pci_read(reg)
#define PCI_WRITE(reg, val) d3d_pci_write(reg, val)

void VFILE_Register(const char *name, const char *path, const char *ext);
bool VFILE_Remove(const char *name, const char *dir);
static void process_d3d_message(Bitu);

// D3D DLL types
enum D3D_DLL_TYPE {
    D3DIMM_DLL,    // D3DImm.dll
    DDRAW_DLL,     // DDraw.dll
    D3D8_DLL,      // D3D8.dll
    D3D9_DLL,      // D3D9.dll
    D3D_DLL_COUNT
};

// GPU types for emulation
enum D3D_GPU_TYPE {
    GPU_GEFORCE4_TI4800,
    GPU_RADEON_8500,
    GPU_MATROX_PARHELIA_512,
    GPU_GEFORCE_FX5700_ULTRA,
    GPU_GEFORCE_9800GT,
    GPU_AUTO_DETECT,
    GPU_TYPE_COUNT
};

// Maximum PCI register count (for all supported cards)
#define MAX_PCI_REGISTERS 256

// D3D block structure for wrapper state
struct D3D_Block {
    bool enabled;
    uint16_t width;
    uint16_t height;
    bool splash;
    class D3D_PageHandler* lfb_pagehandler;
    D3D_GPU_TYPE gpu_type;
    uint32_t vram_size;
    bool vsync;
    bool fullscreen[2];    // current and toggle state
    uint32_t pci_registers[MAX_PCI_REGISTERS]; // PCI registers array
};

// Global D3D state
D3D_Block d3d = {
    false,      // enabled
    0, 0,       // width, height
    false,      // splash
    NULL,       // lfb_pagehandler 
    GPU_AUTO_DETECT, // gpu_type
    256*1024*1024, // vram_size (default 256MB)
    true,       // vsync
    {false, false}, // fullscreen
    {0}         // pci_registers (initialized to 0)
};

// Global DLL handles
#if defined (WIN32)
static HINSTANCE dll_handles[D3D_DLL_COUNT] = {NULL, NULL, NULL, NULL};
#else
static void* dll_handles[D3D_DLL_COUNT] = {NULL, NULL, NULL, NULL};
#endif

// Function table for D3D wrapper
struct D3D_TABLE {
    const char* name;
    const uint8_t parms;
};

// Define function tables for each D3D DLL type
static const D3D_TABLE d3dimm_table[] = {
    { "Direct3DCreate", 4 },
    { "Direct3DCreateDevice", 8 },
    { "Direct3DEnumerate", 4 },
    { "Direct3DGetSWVersion", 0 },
    { NULL, 0 }
};

static const D3D_TABLE ddraw_table[] = {
    { "DirectDrawCreate", 12 },
    { "DirectDrawCreateClipper", 12 },
    { "DirectDrawCreateEx", 16 },
    { "DirectDrawEnumerateA", 8 },
    { "DirectDrawEnumerateExA", 12 },
    { "DirectDrawEnumerateExW", 12 },
    { "DirectDrawEnumerateW", 8 },
    { NULL, 0 }
};

static const D3D_TABLE d3d8_table[] = {
    { "Direct3D8EnableMaximizedWindowedModeShim", 4 },
    { "Direct3DCreate8", 4 },
    { "ValidatePixelShader", 16 },
    { "ValidateVertexShader", 16 },
    { NULL, 0 }
};

static const D3D_TABLE d3d9_table[] = {
    { "Direct3D9EnableMaximizedWindowedModeShim", 4 },
    { "Direct3DCreate9", 4 },
    { "Direct3DCreate9Ex", 8 },
    { NULL, 0 }
};

// Shared memory for D3D calls
static uint32_t d3d_param[20] = {0};

// Pointer to return value
static PhysPt d3d_ret = 0;
static uint16_t d3d_ret_value = 0;

// D3D wrapper buffer size
#define D3D_BUFFER_SIZE (8 * 1024 * 1024)  // 8MB buffer
#define D3D_LFB (0xE0000000)  // Default LFB address
#define D3D_PAGES (1 << 14)   // 64MB total mapping space
#define D3D_PAGE_BITS 2       // 4 buffers (front, back, z, stencil)
#define D3D_BUFFERS 4         // Number of LFB buffer types

// Temporary buffer for D3D operations
static void* d3d_buffer = NULL;
static HostPt hwnd = NULL;
static char lfbacc = 0;

// Debug counters for function calls
#if LOG_D3D
static int D3D_CALL_COUNTS[256] = {0};
#endif

// GPU manufacturer info and PCI device IDs
const char* gpu_manufacturer[] = {
    "NVIDIA",
    "ATI",
    "Matrox",
    "NVIDIA",
    "NVIDIA"
};

// PCI vendor IDs for GPUs
const uint16_t gpu_vendor_ids[] = {
    0x10DE, // NVIDIA
    0x1002, // ATI
    0x102B, // Matrox
    0x10DE, // NVIDIA
    0x10DE  // NVIDIA
};

// PCI device IDs for GPUs
const uint16_t gpu_device_ids[] = {
    0x0280, // GeForce4 Ti 4800
    0x514C, // Radeon 8500
    0x0531, // Matrox Parhelia 512
    0x0342, // GeForce FX 5700 Ultra
    0x0614  // GeForce 9800 GT
};

// PCI register handler for D3D wrapper
uint32_t d3d_pci_read(uint32_t reg) {
    if (reg < MAX_PCI_REGISTERS) {
        return d3d.pci_registers[reg];
    }
    return 0;
}

void d3d_pci_write(uint32_t reg, uint32_t val) {
    if (reg < MAX_PCI_REGISTERS) {
        d3d.pci_registers[reg] = val;
    }
}

// Initialize the PCI registers for the current GPU type
void d3d_init_pci_registers() {
    // Clear all registers
    memset(d3d.pci_registers, 0, sizeof(d3d.pci_registers));
    
    // Setup vendor and device ID
    uint16_t vendor_id = gpu_vendor_ids[d3d.gpu_type];
    uint16_t device_id = gpu_device_ids[d3d.gpu_type];
    
    // PCI header registers
    d3d.pci_registers[0x00] = vendor_id | (device_id << 16);
    d3d.pci_registers[0x01] = 0x00100006; // Command and status
    d3d.pci_registers[0x02] = 0x03000000; // Class code (display controller, VGA compatible)
    d3d.pci_registers[0x03] = 0x00000000; // BIST, header type, latency timer, cache line size
    
    // Base Address Registers (BARs)
    d3d.pci_registers[0x04] = 0xF0000000; // Memory-mapped BAR 0 (framebuffer)
    d3d.pci_registers[0x05] = 0x00000000; // Memory-mapped BAR 1 (IO)
    d3d.pci_registers[0x06] = 0xF2000000; // Memory-mapped BAR 2 (MMIO)
    d3d.pci_registers[0x07] = 0x00000000; // Memory-mapped BAR 3
    d3d.pci_registers[0x08] = 0x00000000; // Memory-mapped BAR 4
    d3d.pci_registers[0x09] = 0x00000000; // Memory-mapped BAR 5
    
    // Subsystem ID
    d3d.pci_registers[0x0B] = vendor_id | (device_id << 16);
    
    // Setup card-specific registers for each GPU
    switch (d3d.gpu_type) {
        case GPU_GEFORCE4_TI4800:
            // NVIDIA-specific registers
            d3d.pci_registers[0x10] = 0x10DE0280;
            d3d.pci_registers[0x40] = 0x00000000; // NV4 specific
            break;
        
        case GPU_RADEON_8500:
            // ATI-specific registers
            d3d.pci_registers[0x10] = 0x1002514C;
            d3d.pci_registers[0x58] = 0x00030000; // ATI config register
            break;
            
        case GPU_MATROX_PARHELIA_512:
            // Matrox-specific registers
            d3d.pci_registers[0x10] = 0x102B0531;
            d3d.pci_registers[0x50] = 0x00000001; // Matrox config register
            break;
            
        case GPU_GEFORCE_FX5700_ULTRA:
            // NVIDIA FX-specific registers
            d3d.pci_registers[0x10] = 0x10DE0342;
            d3d.pci_registers[0x40] = 0x00000001; // NV30 specific
            break;
            
        case GPU_GEFORCE_9800GT:
            // NVIDIA G92-specific registers
            d3d.pci_registers[0x10] = 0x10DE0614;
            d3d.pci_registers[0x40] = 0x00000002; // G92 specific
            break;
            
        default:
            break;
    }
}

// Functions for D3D port read/write
static Bitu read_d3d(Bitu port, Bitu iolen) {
    (void)port;
    (void)iolen;
    Bitu r=d3d_ret_value;
#if LOG_D3D
    if(d3d_ret_value == D3D_OK)
        LOG_MSG("D3D: Port read. Return address: 0x%x, value: %d", d3d_ret, mem_readd(d3d_ret));
    else if(d3d_ret_value == D3D_FAIL)
        LOG_MSG("D3D: Port read. Return address: 0x%x, value: %d. Writing D3D_FAIL to port", d3d_ret, mem_readd(d3d_ret));
    else
        LOG_MSG("D3D: Port read. Returning %hu", d3d_ret_value);
#endif

    d3d_ret_value = d3d_ret_value >> 8;

    return r;
}

static void write_d3d(Bitu port, Bitu val, Bitu iolen) {
    (void)port;
    (void)iolen;
    static uint16_t d3d_segment = 0;

    d3d_ret = 0;
    d3d_ret_value = D3D_FAIL;

    // Allocate shared memory (80 bytes)
    if(val > 0xFF) {
        if(d3d_segment==0 && !DOS_GetMemory_unmapped) {
            d3d_segment=DOS_GetMemory(5);
#if LOG_D3D
            LOG_MSG("D3D: Memory allocated at 0x%x (segment: %hu)", d3d_segment<<4, d3d_segment);
#endif
        }
        // Even if DOS_GetMemory failed, still return segment to avoid crashing
        d3d_ret_value = d3d_segment;
        LOG_MSG("D3D: Activated");
        return;
    }

    // Safety check - make sure segment is valid
    if (d3d_segment == 0) {
        LOG_MSG("D3D: Error - Memory segment not allocated");
        return;
    }

    // Process function parameters (80 bytes)
    // Safely read memory, checking for valid segment
    PhysPt physAddress = PhysMake(d3d_segment, 0);
    if (physAddress != 0) {
        MEM_BlockRead32(physAddress, d3d_param, 80);
        process_d3d_message(val);
    }
}

static void statWMInfo(void) {
    // Initialize hwnd to NULL to avoid using uninitialized value
    hwnd = NULL;
    
    // Get hwnd information
    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
#if defined(C_SDL2)
    if(SDL_GetWindowWMInfo(sdl.window, &wmi)) {
# if defined (WIN32)
        hwnd = (HostPt)wmi.info.win.window;
# elif defined (C_X11) && defined(LINUX)
        hwnd = (HostPt)wmi.info.x11.window;
# endif
#else
    SDL_VERSION(&wmi.version);
    if(SDL_GetWMInfo(&wmi)) {
# if defined (WIN32)
        hwnd = (HostPt)wmi.window;
# elif defined (C_X11) && defined(LINUX)
        hwnd = (HostPt)wmi.info.x11.window;
# endif
#endif
    } else {
        LOG_MSG("SDL: Error retrieving window information");
    }
}

class D3D_PageHandler : public PageHandler {
private:
    PhysPt base_addr[D3D_BUFFERS];  // D3D_LFB physical address
    PhysPt lin_addr[D3D_BUFFERS];   // D3D_LFB linear address
    HostPt lfb_addr[D3D_BUFFERS];   // Address offset from base_addr for the readable/writable buffers

    /* Calculate physical address for access */
    HostPt LFB_getAddr(PhysPt addr) {
        Bitu buffer_idx = ((addr - base_addr[0]) >> 12) >> D3D_PAGE_BITS;
        if (buffer_idx >= D3D_BUFFERS) {
            buffer_idx = 0; // Fail safe
        }
        if (!lfb_addr[buffer_idx]) {
            // Safety check - shouldn't happen, but avoid crash if it does
            LOG_MSG("D3D: Warning - null lfb_addr for buffer %d", buffer_idx);
            return NULL;
        }
        return lfb_addr[buffer_idx] + addr;
    }

public:
    uint8_t locked[D3D_BUFFERS];

    D3D_PageHandler(HostPt addr, PhysPt phyaddr = D3D_LFB) {
        // Initialize arrays to safe values
        for (int i = 0; i < D3D_BUFFERS; i++) {
            locked[i] = 0;
            base_addr[i] = 0;
            lin_addr[i] = 0;
            lfb_addr[i] = NULL;
        }

        if (addr == NULL) {
            LOG_MSG("D3D: NULL address passed to pagehandler!");
            return;
        }

        /* Set base addresses */
        for (int i = 0; i < D3D_BUFFERS; i++) {
            locked[i] = 0;
            base_addr[i] = phyaddr;
            phyaddr += ((1 << D3D_PAGE_BITS) << 12);

            /* store offset from base address */
            lfb_addr[i] = addr - base_addr[i];
        }

        flags = PFLAG_READABLE | PFLAG_WRITEABLE | PFLAG_NOCODE;
        PAGING_UnlinkPages(base_addr[0] >> 12, D3D_PAGES);
#if LOG_D3D
        LOG_MSG("D3D: D3D_PageHandler installed at 0x%x", base_addr[0]);
#endif
    }

    ~D3D_PageHandler() {
#if LOG_D3D
        LOG_MSG("D3D: Resetting page handler");
#endif
        if (base_addr[0] != 0) {
            PAGING_UnlinkPages(base_addr[0] >> 12, D3D_PAGES);
        }
    }

    void SetLFBAddr(HostPt addr, Bitu buffer) {
        if (buffer >= D3D_BUFFERS || base_addr[buffer] == 0) return;

        addr = addr - base_addr[buffer];  /* Calculate offset for current buffer */

        if (addr != lfb_addr[buffer]) {
            lfb_addr[buffer] = addr;
#if LOG_D3D
            LOG_MSG("D3D: LFB for buffer %d offset set to 0x%p (addr: 0x%p), clear TLB", buffer, lfb_addr[buffer], addr + base_addr[buffer]);
#endif
            PAGING_UnlinkPages(base_addr[buffer] >> 12, 1 << D3D_PAGE_BITS);
        }
    }

    PhysPt GetPhysPt(Bitu buffer = 0) {
        if (buffer >= D3D_BUFFERS) return base_addr[0];
        return base_addr[buffer];
    }

    void SetLinPt(PhysPt linaddr) {
        for (int i = 0; i < D3D_BUFFERS; i++) {
            lin_addr[i] = linaddr;
            linaddr += ((1 << D3D_PAGE_BITS) << 12);
        }
    }

    PhysPt GetLinPt(Bitu buffer = 0) {
        if (buffer >= D3D_BUFFERS) return lin_addr[0];
        return lin_addr[buffer];
    }

    uint8_t readb(PhysPt addr) override {
        HostPt hostAddr = LFB_getAddr(addr);
        if (!hostAddr) return 0;
        return *(uint8_t*)hostAddr;
    }

    uint16_t readw(PhysPt addr) override {
        HostPt hostAddr = LFB_getAddr(addr);
        if (!hostAddr) return 0;
        return *(uint16_t*)hostAddr;
    }

    uint32_t readd(PhysPt addr) override {
        HostPt hostAddr = LFB_getAddr(addr);
        if (!hostAddr) return 0;
        return *(uint32_t*)hostAddr;
    }

    void writeb(PhysPt addr, uint8_t val) override {
        HostPt hostAddr = LFB_getAddr(addr);
        if (!hostAddr) return;
        *(uint8_t*)hostAddr = val;
    }

    void writew(PhysPt addr, uint16_t val) override {
        HostPt hostAddr = LFB_getAddr(addr);
        if (!hostAddr) return;
        *(uint16_t*)hostAddr = val;
    }

    void writed(PhysPt addr, uint32_t val) override {
        HostPt hostAddr = LFB_getAddr(addr);
        if (!hostAddr) return;
        *(uint32_t*)hostAddr = val;
    }

    HostPt GetHostReadPt(PageNum phys_page) override {
        if (base_addr[0] == 0) return NULL;
        
        Bitu buffer = (((phys_page << 12) - base_addr[0]) >> 12) >> D3D_PAGE_BITS;
        if (buffer >= D3D_BUFFERS) buffer = 0;
        
#if LOG_D3D
        // This only makes sense if full lfb access is used...
        if (!locked[buffer]) LOG_MSG("D3D: Read from unlocked LFB at: 0x%x", phys_page << 12);
#endif
        return lfb_addr[buffer] ? (lfb_addr[buffer] + (phys_page << 12)) : NULL;
    }

    HostPt GetHostWritePt(PageNum phys_page) override {
        if (base_addr[0] == 0) return NULL;
        
        Bitu buffer = (((phys_page << 12) - base_addr[0]) >> 12) >> D3D_PAGE_BITS;
        if (buffer >= D3D_BUFFERS) buffer = 0;
        
#if LOG_D3D
        // This only makes sense if full lfb access is used...
        if (!locked[buffer]) LOG_MSG("D3D: Write to unlocked LFB at: 0x%x", phys_page << 12);
#endif
        return lfb_addr[buffer] ? (lfb_addr[buffer] + (phys_page << 12)) : NULL;
    }
};

bool addD3DOvl = false;

// Load dgVoodoo2 DLL
bool load_dgVoodoo2_dll(D3D_DLL_TYPE type) {
    const char* dll_names[D3D_DLL_COUNT] = {
        "D3DImm.dll", "DDraw.dll", "D3D8.dll", "D3D9.dll"
    };
    
    const char* wrapper_names[D3D_DLL_COUNT] = {
        "dgVoodooD3DImm.dll", "dgVoodooDDraw.dll", "dgVoodooD3D8.dll", "dgVoodooD3D9.dll"
    };
    
    // Return true if already loaded
    if (dll_handles[type] != NULL) {
        return true;
    }
    
    // Try to load the dgVoodoo wrapper DLL
#if defined(WIN32)
    dll_handles[type] = LoadLibrary(wrapper_names[type]);
#elif defined(MACOSX)
    char wrapper_dylib[64];
    snprintf(wrapper_dylib, sizeof(wrapper_dylib), "lib%s.dylib", wrapper_names[type]);
    dll_handles[type] = dlopen(wrapper_dylib, RTLD_NOW);
#else
    char wrapper_so[64];
    snprintf(wrapper_so, sizeof(wrapper_so), "lib%s.so", wrapper_names[type]);
    dll_handles[type] = dlopen(wrapper_so, RTLD_NOW);
#endif

    if (dll_handles[type] == NULL) {
        LOG_MSG("D3D: Failed to load %s wrapper, direct3d emulation may not function properly", dll_names[type]);
        return false;
    }

    LOG_MSG("D3D: Successfully loaded %s wrapper", dll_names[type]);
    return true;
}

void D3D_ResetScreen(bool update)
{
#if LOG_D3D
    LOG_MSG("D3D: ResetScreen");
#endif
    VGA_SetOverride(true);
    GFX_Stop();

    if (d3d.width && update) {
#if defined(C_SDL2)
        void GFX_SetResizeable(bool enable);
        GFX_SetResizeable(true);
        SDL_Window* GFX_SetSDLWindowMode(uint16_t width, uint16_t height, SCREEN_TYPES screenType);
        sdl.window = GFX_SetSDLWindowMode(d3d.width, d3d.height, sdl.desktop.want_type == SCREEN_OPENGL ? SCREEN_OPENGL : SCREEN_SURFACE);
        if (sdl.window != NULL) sdl.surface = SDL_GetWindowSurface(sdl.window);
#else
        SDL_Surface* SDL_SetVideoMode(int width, int height, int bpp, uint32_t flags);
        sdl.surface = SDL_SetVideoMode(d3d.width, d3d.height, 0, (d3d.fullscreen[0] ? SDL_FULLSCREEN : 0) | SDL_ANYFORMAT);
#endif
    }
    ApplyPreventCap();
}

void D3D_DisableScreen(void)
{
    d3d.enabled = false;  /* if not disabled, GFX_ResetScreen() will call D3D_ResetScreen() */
    VGA_SetOverride(false);
    GFX_ResetScreen();
}

// Function to initialize D3D module
class D3D: public Module_base {
private:
    AutoexecObject autoexecline;
    // D3D port
    Bitu d3d_base;
public:
    D3D(Section* configuration): Module_base(configuration), d3d_base(0) {
        addD3DOvl = false;
        Section_prop* section = static_cast<Section_prop*>(configuration);

        if (!section || !section->Get_bool("d3d")) return;
        
        // LFB settings
        std::string str = section->Get_string("lfb");
        lowcase(str);
        if (str == "none") {
            LOG_MSG("D3D: Disabled LFB access");
            lfbacc = 0;
        } else if (str == "read_noaux") {
            LOG_MSG("D3D: LFB access: read-only (no aux)");
            lfbacc = 1;
        } else if (str == "write_noaux") {
            LOG_MSG("D3D: LFB access: write-only (no aux)");
            lfbacc = 2;
        } else if (str == "full_noaux") {
            LOG_MSG("D3D: LFB access: read-write (no aux)");
            lfbacc = 3;
        } else if (str == "read") {
            LOG_MSG("D3D: LFB access: read-only");
            lfbacc = 5;
        } else if (str == "write") {
            LOG_MSG("D3D: LFB access: write-only");
            lfbacc = 6;
        } else {
            LOG_MSG("D3D: LFB access: read-write");
            lfbacc = 7;
        }

        // GPU type
        str = section->Get_string("gpu");
        lowcase(str);
        if (str == "geforce4_ti4800") {
            d3d.gpu_type = GPU_GEFORCE4_TI4800;
            d3d.vram_size = 128 * 1024 * 1024;  // 128MB
        } else if (str == "radeon_8500") {
            d3d.gpu_type = GPU_RADEON_8500;
            d3d.vram_size = 64 * 1024 * 1024;   // 64MB
        } else if (str == "matrox_parhelia") {
            d3d.gpu_type = GPU_MATROX_PARHELIA_512;
            d3d.vram_size = 512 * 1024 * 1024;  // 512MB
        } else if (str == "geforce_fx5700") {
            d3d.gpu_type = GPU_GEFORCE_FX5700_ULTRA;
            d3d.vram_size = 256 * 1024 * 1024;  // 256MB
        } else if (str == "geforce_9800gt") {
            d3d.gpu_type = GPU_GEFORCE_9800GT;
            d3d.vram_size = 512 * 1024 * 1024;  // 512MB
        } else {
            d3d.gpu_type = GPU_AUTO_DETECT;
            d3d.vram_size = 256 * 1024 * 1024;  // 256MB default
            LOG_MSG("D3D: Auto-detecting GPU type");
        }

        d3d.vsync = section->Get_bool("vsync");
        d3d.fullscreen[0] = d3d.fullscreen[1] = section->Get_bool("fullscreen");

        // Initialize PCI registers based on the GPU type
        d3d_init_pci_registers();

        // Try to preload required DLLs but don't fail if they're not found
        try {
            load_dgVoodoo2_dll(D3DIMM_DLL);
            load_dgVoodoo2_dll(DDRAW_DLL);
            load_dgVoodoo2_dll(D3D8_DLL);
            load_dgVoodoo2_dll(D3D9_DLL);
        } catch (...) {
            LOG_MSG("D3D: Exception while loading dgVoodoo2 DLLs");
        }

        // Allocate buffer for D3D operations
        try {
            d3d_buffer = (void*)malloc(D3D_BUFFER_SIZE);
            if (d3d_buffer == NULL) {
                LOG_MSG("D3D: Unable to allocate D3D buffer memory, D3D disabled");
                return;
            }

            // Initialize buffer with zeros to avoid undefined behavior
            memset(d3d_buffer, 0, D3D_BUFFER_SIZE);
            
            d3d.lfb_pagehandler = new D3D_PageHandler((HostPt)d3d_buffer);
            if (!d3d.lfb_pagehandler) {
                LOG_MSG("D3D: Failed to install page handler, D3D disabled!");
                free(d3d_buffer); d3d_buffer = NULL;
                return;
            }
        } catch (...) {
            LOG_MSG("D3D: Exception while setting up D3D buffer");
            if (d3d_buffer) {
                free(d3d_buffer);
                d3d_buffer = NULL;
            }
            return;
        }

#if LOG_D3D
        memset(D3D_CALL_COUNTS, 0, sizeof(D3D_CALL_COUNTS));
#endif

        // Define I/O port for D3D wrapper
        d3d_base = 0x666;  // D3D port

        IO_RegisterReadHandler(d3d_base, read_d3d, IO_MB);
        IO_RegisterWriteHandler(d3d_base, write_d3d, IO_MB);

        ostringstream temp;
        temp << "@SET D3D=" << hex << d3d_base << ends;

        try {
            autoexecline.Install(temp.str());
        } catch (...) {
            LOG_MSG("D3D: Failed to install autoexec line");
        }
        
        d3d.splash = section->Get_bool("splash");
        d3d.enabled = false; // Will be enabled when a D3D app is detected

        addD3DOvl = true;
    }

    ~D3D() {
        if (d3d.enabled) {
            try {
                D3D_DisableScreen();
            } catch (...) {
                LOG_MSG("D3D: Exception while disabling screen");
            }
            d3d.enabled = false;
        }

        SAFE_DELETE(d3d.lfb_pagehandler);
        
        if (d3d_buffer) {
            free(d3d_buffer); d3d_buffer = NULL;
        }
        
        if (d3d_base) {
            IO_FreeReadHandler(d3d_base, IO_MB);
            IO_FreeWriteHandler(d3d_base, IO_MB);
        }

        // Clean up dll handles
        for (int i = 0; i < D3D_DLL_COUNT; i++) {
            if (dll_handles[i]) {
#if defined (WIN32)
                FreeLibrary(dll_handles[i]);
#else
                dlclose(dll_handles[i]);
#endif
                dll_handles[i] = NULL;
            }
        }

        // Safely remove any virtual files if they were added
        if (addD3DOvl) {
            LOG_MSG("D3D: Cleaning up virtual files");
            // Instead of trying to use VFILE_Remove, we just log that we're
            // done with the virtual files. The files will be cleaned up when
            // DOSBox-X exits anyway.
            addD3DOvl = false;
        }
    }
};

static D3D* test_d3d = NULL;

void D3D_ShutDown(Section* sec) {
    (void)sec; // UNUSED
    if (test_d3d) {
        delete test_d3d;
        test_d3d = NULL;
    }
}

void D3D_PowerOn(Section* sec) {
    if (test_d3d) {
        D3D_ShutDown(sec);
    }
    try {
        test_d3d = new D3D(sec);
    } catch (...) {
        LOG_MSG("D3D: Exception during D3D initialization");
        test_d3d = NULL;
    }
}

void D3D_Init() {
    try {
        Section* sec = control->GetSection("d3d");
        if (sec) {
            test_d3d = new D3D(sec);
            if (test_d3d) {
                AddExitFunction(AddExitFunctionFuncPair(D3D_ShutDown), true);
            }
        }
    } catch (...) {
        LOG_MSG("D3D: Failed to initialize D3D module");
    }
}

// Function that processes D3D wrapper messages
static void process_d3d_message(Bitu value) {
    // Store the return address for the wrapper
    d3d_ret = d3d_param[0];

    // Default return is fail
    d3d_ret_value = D3D_FAIL;

    // Basic validation - the message value should be within range
    if (value > 0xFF) {
        LOG_MSG("D3D: Invalid function call code 0x%x", value);
        return;
    }

#if LOG_D3D
    LOG_MSG("D3D: Processing call (%d), return address: 0x%x", value, d3d_ret);
    if (value < 256) {
        D3D_CALL_COUNTS[value]++;
    }
#endif

    // Handle different D3D API calls based on the value
    try {
        switch (value) {
            case 0x01:  // Initialize D3D
                if (d3d.enabled) {
                    d3d_ret_value = D3D_OK;
                    break;  // Already initialized
                }
                
                // Initialize D3D subsystem
                d3d.enabled = true;
                d3d.width = d3d_param[1];
                d3d.height = d3d_param[2];
                
                LOG_MSG("D3D: Initializing with resolution %dx%d", d3d.width, d3d.height);
                
                // Resize window and configure screen
                D3D_ResetScreen(true);

                // Get window handle
                statWMInfo();

                // Make sure we have a valid page handler
                if (d3d.lfb_pagehandler) {
                    // Send LFB to the D3D wrapper
                    mem_writed(d3d_param[3], d3d.lfb_pagehandler->GetPhysPt());
                    mem_writed(d3d_param[4], D3D_PAGES);
                    
                    // Set up PCI information
                    mem_writed(d3d_param[5], d3d.gpu_type);
                    mem_writed(d3d_param[6], d3d.vram_size);

                    // Return success
                    d3d_ret_value = D3D_OK;
                }
                break;

            case 0x02:  // Shutdown D3D
                if (d3d.enabled) {
                    LOG_MSG("D3D: Shutting down D3D subsystem");
                    D3D_DisableScreen();
                    d3d.enabled = false;
                }
                d3d_ret_value = D3D_OK;
                break;

            case 0x10:  // Lock framebuffer
                if (!d3d.enabled || !d3d.lfb_pagehandler) break;
                
                {
                    Bitu buffer = d3d_param[1];
                    if (buffer >= D3D_BUFFERS) {
                        LOG_MSG("D3D: Invalid buffer passed in LFB lock (%lu)", buffer);
                        break;
                    }
                    
                    // Check LFB access permissions
                    Bitu access_mode = d3d_param[2] & 0x03;  // 1=read, 2=write, 3=read-write
                    if ((access_mode > 0) && ((access_mode & lfbacc) == 0)) {
                        LOG_MSG("D3D: LFB access denied (mode %d)", access_mode);
                        break;
                    }
                    
                    d3d.lfb_pagehandler->locked[buffer]++;
                    
                    // Return linear address pointer
                    mem_writed(d3d_ret, d3d.lfb_pagehandler->GetLinPt(buffer));
                    
#if LOG_D3D
                    LOG_MSG("D3D: LFB lock buffer (%d), returning address 0x%x", 
                        buffer, d3d.lfb_pagehandler->GetLinPt(buffer));
#endif
                    d3d_ret_value = D3D_OK;
                }
                break;
                
            case 0x11:  // Unlock framebuffer
                if (!d3d.enabled || !d3d.lfb_pagehandler) break;
                
                {
                    Bitu buffer = d3d_param[1];
                    if (buffer >= D3D_BUFFERS) {
                        LOG_MSG("D3D: Invalid buffer passed in LFB unlock (%lu)", buffer);
                        break;
                    }
                    
                    if (d3d.lfb_pagehandler->locked[buffer] > 0) {
                        d3d.lfb_pagehandler->locked[buffer]--;
#if LOG_D3D
                        LOG_MSG("D3D: LFB unlock buffer (%d), %d locks remaining", 
                            buffer, d3d.lfb_pagehandler->locked[buffer]);
#endif
                        d3d_ret_value = D3D_OK;
                    }
                }
                break;
                
            case 0x20:  // PCI register read
                {
                    Bitu reg = d3d_param[1];
                    if (reg >= MAX_PCI_REGISTERS) {
                        LOG_MSG("D3D: Invalid PCI register read: 0x%x", reg);
                        break;
                    }
                    
                    uint32_t value = PCI_READ(reg);
                    mem_writed(d3d_ret, value);
#if LOG_D3D
                    LOG_MSG("D3D: PCI read register 0x%x = 0x%08x", reg, value);
#endif
                    d3d_ret_value = D3D_OK;
                }
                break;
                
            case 0x21:  // PCI register write
                {
                    Bitu reg = d3d_param[1];
                    uint32_t value = d3d_param[2];
                    
                    if (reg >= MAX_PCI_REGISTERS) {
                        LOG_MSG("D3D: Invalid PCI register write: 0x%x", reg);
                        break;
                    }
                    
#if LOG_D3D
                    LOG_MSG("D3D: PCI write register 0x%x = 0x%08x", reg, value);
#endif
                    PCI_WRITE(reg, value);
                    d3d_ret_value = D3D_OK;
                }
                break;
                
            case 0x30:  // Get D3D wrapper info
                {
                    if (d3d.gpu_type >= GPU_TYPE_COUNT) {
                        d3d.gpu_type = GPU_AUTO_DETECT;
                    }
                    
                    // Return D3D wrapper version and GPU information
                    // Ensure buffer exists before writing to it
                    if (d3d_param[2] != 0) {
                        char info[256];
                        snprintf(info, sizeof(info), "dgVoodoo2 D3D Wrapper - Emulating %s %s with %dMB VRAM",
                            gpu_manufacturer[d3d.gpu_type],
                            d3d_param[1] ? "GPU" : "Display Adapter",
                            d3d.vram_size / (1024 * 1024));
                        
                        // Copy string to the return buffer
                        uint32_t dst = d3d_param[2];
                        if (dst != 0) {
                            for (size_t i = 0; i < strlen(info) + 1; i++) {
                                mem_writeb(dst++, info[i]);
                            }
                        }
                    }
                    d3d_ret_value = D3D_OK;
                }
                break;
                
            case 0xFE:  // Custom Debug Message
                {
                    LOG_MSG("D3D: Debug message from wrapper, param1=%d, param2=%d", 
                        d3d_param[1], d3d_param[2]);
                    d3d_ret_value = D3D_OK;
                }
                break;
                
            case 0xFF:  // Get API Statistics
                if (d3d_ret) {
#if LOG_D3D
                    uint32_t functionId = d3d_param[1];
                    if (functionId < 256) {
                        mem_writed(d3d_ret, D3D_CALL_COUNTS[functionId]);
                    } else {
                        mem_writed(d3d_ret, 0);
                    }
                    d3d_ret_value = D3D_OK;
#else
                    mem_writed(d3d_ret, 0);  // Stats not tracked when logging is disabled
                    d3d_ret_value = D3D_OK;
#endif
                }
                break;
                
            default:
                LOG_MSG("D3D: Unhandled D3D function code %d", value);
                break;
        }
    } catch (...) {
        LOG_MSG("D3D: Exception in process_d3d_message for function %d", value);
        d3d_ret_value = D3D_FAIL;
    }
}