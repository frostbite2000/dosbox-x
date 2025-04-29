/*
 *  Copyright (C) 2002-2021  The DOSBox Team
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

/* NTS: Hardware notes
 *
 *   NVIDIA RIVA 128 (NV3):
 *
 *   PCI IDs: 
 *     Vendor: 0x12D2 (NVIDIA)
 *     Device: 0x0018 (RIVA 128)
 *
 *   Classic VGA compatible with extended SVGA capabilities.
 *   Features 4MB of SGRAM memory.
 *   Supports resolutions up to 1600x1200 with 16-bit color.
 *   Hardware cursor and hardware-accelerated 2D functions.
 *   Supports DirectX 5.0 for early 3D gaming.
 */

#define VGA_INTERNAL

#include <assert.h>
#include "dosbox.h"
#include "setup.h"
#include "vga.h"
#include "mem.h"
#include "pci_bus.h"
#include "logging.h"
#include "control.h"

// NV3 PCI Identifiers
#define NV3_VENDOR_ID 0x12D2
#define NV3_DEVICE_ID 0x0018
#define NV3_LFB_BASE 0xE0000000      // Default Linear Framebuffer Base Address

// NV3 MMIO Register Regions (offsets from MMIO base)
#define NV3_PMC     0x000000    // Master Control
#define NV3_PBUS    0x001000    // Bus Control
#define NV3_PFIFO   0x002000    // FIFO Command Submission
#define NV3_PDMA    0x003000    // DMA Engine
#define NV3_PTIMER  0x009000    // Time Measurement
#define NV3_PFB     0x100000    // Framebuffer Control
#define NV3_PRAMHT  0x110000    // RAMHT Control
#define NV3_PRAMFC  0x120000    // RAMFC Control
#define NV3_PRAMRO  0x130000    // RAMRO Control
#define NV3_PRAMDAC 0x680000    // Video DAC
#define NV3_PRMDIO  0x6C0000    // Memory-mapped I/O
#define NV3_PRMVIO  0x0C0000    // Traditional VGA I/O
#define NV3_PRM     0x6D0000    // VGA Sequencer and Graph Controller
#define NV3_PCRTC   0x600000    // CRTC

// PCRTC Registers (from envytools documentation)
#define NV3_PCRTC_INTR                   0x600100
#define NV3_PCRTC_INTR_0                 0x600104
#define NV3_PCRTC_CONFIG                 0x600804
#define NV3_PCRTC_START                  0x600800
#define NV3_PCRTC_CURSOR_CONFIG          0x600810
#define NV3_PCRTC_CURSOR_ADR_LO          0x600814
#define NV3_PCRTC_CURSOR_ADR_HI          0x600818
#define NV3_PCRTC_INDEX                  0x6013D4
#define NV3_PCRTC_DATA                   0x6013D5
#define NV3_PCRTC_GPIO                   0x60081C
#define NV3_PCRTC_CURSOR_POS             0x600820

// PRAMDAC Registers
#define NV3_PRAMDAC_NVPLL_COEFF          0x680500
#define NV3_PRAMDAC_MPLL_COEFF           0x680504
#define NV3_PRAMDAC_VPLL_COEFF           0x680508
#define NV3_PRAMDAC_PLL_TEST_COUNTER     0x68050C
#define NV3_PRAMDAC_GENERAL_CONTROL      0x680600
#define NV3_PRAMDAC_PALETTE_DATA         0x6813C0    // 8-bit palette entry
#define NV3_PRAMDAC_PALETTE_DATA_ALT     0x6813C8    // 8-bit palette entry with doubling

// Traditional VGA registers
#define NV3_PVGA_ATTR_INDEX              0x6C03C0
#define NV3_PVGA_ATTR_DATA_W             0x6C03C0
#define NV3_PVGA_ATTR_DATA_R             0x6C03C1
#define NV3_PVGA_MISC_W                  0x6C03C2
#define NV3_PVGA_MISC_R                  0x6C03CC
#define NV3_PVGA_STATUS_0                0x6C03C2
#define NV3_PVGA_STATUS_1                0x6C03DA
#define NV3_PVGA_SEQ_INDEX               0x6C03C4
#define NV3_PVGA_SEQ_DATA                0x6C03C5
#define NV3_PVGA_GFX_INDEX               0x6C03CE
#define NV3_PVGA_GFX_DATA                0x6C03CF

// Memory-mapped VGA register file offsets (relative to PRMVIO)
#define NV3_PRMVIO_MISC_W                0x000C2
#define NV3_PRMVIO_MISC_R                0x000CC
#define NV3_PRMVIO_SEQ_INDEX             0x000C4
#define NV3_PRMVIO_SEQ_DATA              0x000C5
#define NV3_PRMVIO_GFX_INDEX             0x000CE
#define NV3_PRMVIO_GFX_DATA              0x000CF
#define NV3_PRMVIO_ATTR_INDEX            0x000C0
#define NV3_PRMVIO_ATTR_DATA_W           0x000C0
#define NV3_PRMVIO_ATTR_DATA_R           0x000C1
#define NV3_PRMVIO_CRTC_INDEX            0x003D4
#define NV3_PRMVIO_CRTC_DATA             0x003D5

// NV3 CRTC Standard Registers (indexed via CRTC_INDEX/CRTC_DATA)
#define NV3_CRTC_HORIZ_TOTAL             0x00
#define NV3_CRTC_HORIZ_DISPLAY_END       0x01
#define NV3_CRTC_HORIZ_BLANK_START       0x02
#define NV3_CRTC_HORIZ_BLANK_END         0x03
#define NV3_CRTC_HORIZ_SYNC_START        0x04
#define NV3_CRTC_HORIZ_SYNC_END          0x05
#define NV3_CRTC_VERT_TOTAL              0x06
#define NV3_CRTC_OVERFLOW                0x07
#define NV3_CRTC_PRESET_ROW_SCAN         0x08
#define NV3_CRTC_MAX_SCAN_LINE           0x09
#define NV3_CRTC_CURSOR_START            0x0A
#define NV3_CRTC_CURSOR_END              0x0B
#define NV3_CRTC_START_ADDR_HIGH         0x0C
#define NV3_CRTC_START_ADDR_LOW          0x0D
#define NV3_CRTC_CURSOR_LOC_HIGH         0x0E
#define NV3_CRTC_CURSOR_LOC_LOW          0x0F
#define NV3_CRTC_VERT_RETRACE_START      0x10
#define NV3_CRTC_VERT_RETRACE_END        0x11
#define NV3_CRTC_VERT_DISPLAY_END        0x12
#define NV3_CRTC_OFFSET                  0x13
#define NV3_CRTC_UNDERLINE_LOC           0x14
#define NV3_CRTC_VERT_BLANK_START        0x15
#define NV3_CRTC_VERT_BLANK_END          0x16
#define NV3_CRTC_MODE_CONTROL            0x17
#define NV3_CRTC_LINE_COMPARE            0x18

// NV3 CRTC Extended Registers
#define NV3_CRTC_REPAINT0                0x19    // Extended offset and start address
#define NV3_CRTC_REPAINT1                0x1A    // Various flags
#define NV3_CRTC_FIFO_CONTROL            0x1B    // FIFO control
#define NV3_CRTC_FIFO                    0x20    // FIFO watermark
#define NV3_CRTC_EXTRA                   0x25    // Extra bits
#define NV3_CRTC_PIXEL                   0x28    // Pixel format
#define NV3_CRTC_HORIZ_EXTRA             0x2D    // Horizontal extra bits
#define NV3_CRTC_GRCURSOR0               0x30    // Graphic cursor 0
#define NV3_CRTC_GRCURSOR1               0x31    // Graphic cursor 1
#define NV3_CRTC_REVISION_ID             0x34    // Revision ID

// NV3 CRTC REPAINT1 register bits
#define NV3_REPAINT1_PALETTE_WIDTH       (1 << 1)  // 0: 8-bit, 1: 6-bit
#define NV3_REPAINT1_LARGE_SCREEN        (1 << 2)  // 0: >= 1280, 1: disable
#define NV3_REPAINT1_COMPATIBLE_TEXT     (1 << 4)  // 0: disable, 1: enable
#define NV3_REPAINT1_VSYNC_DISABLE       (1 << 6)  // 0: enable, 1: disable
#define NV3_REPAINT1_HSYNC_DISABLE       (1 << 7)  // 0: enable, 1: disable

// NV3 CRTC FIFO_CONTROL register bits
#define NV3_FIFO_CONTROL_BURST_LENGTH_8   0x0
#define NV3_FIFO_CONTROL_BURST_LENGTH_32  0x1
#define NV3_FIFO_CONTROL_BURST_LENGTH_64  0x2
#define NV3_FIFO_CONTROL_BURST_LENGTH_128 0x3
#define NV3_FIFO_CONTROL_BURST_LENGTH_256 0x4
#define NV3_FIFO_CONTROL_UNDERFLOW_WARN   (1 << 7)

// NV3 CRTC FIFO register bits
#define NV3_FIFO_WATERMARK_MASK           0x3F
#define NV3_FIFO_RESET                    (1 << 7)

// NV3 CRTC PIXEL register bits
#define NV3_PIXEL_FORMAT_VGA             0x0
#define NV3_PIXEL_FORMAT_8BPP            0x1
#define NV3_PIXEL_FORMAT_16BPP           0x2
#define NV3_PIXEL_FORMAT_32BPP           0x3
#define NV3_PIXEL_TV_MODE_NTSC           0x0
#define NV3_PIXEL_TV_MODE_PAL            (1 << 6)
#define NV3_PIXEL_MODE_TV                (1 << 7)
#define NV3_PIXEL_MODE_VGA               0x0

#define NV3_PLL_REF_FREQ 13500000UL      // 13.5 MHz reference clock

// NV3 PLL control macros
#define NV3_PLL_N_MASK 0xFF              // 8-bit N value
#define NV3_PLL_M_MASK 0xFF              // 8-bit M value
#define NV3_PLL_P_MASK 0x0F              // 4-bit P value
#define NV3_PLL_N_SHIFT 0
#define NV3_PLL_M_SHIFT 8
#define NV3_PLL_P_SHIFT 16

// Maximum number of NV3 registers
#define NV3_CRTC_MAX_REG    0x40         // Maximum register index

typedef struct {
    // NV3 state variables
    bool enabled;                              // Is the card enabled
    bool hardware_cursor_active;               // Is hardware cursor enabled
    uint32_t mmio_base_addr;                   // MMIO base address
    uint32_t fb_base_addr;                     // Framebuffer base address
    uint32_t linear_fb_addr;                   // Linear framebuffer address
    uint8_t crtc_reg[NV3_CRTC_MAX_REG];        // CRTC registers
    
    // Display/CRTC state
    uint16_t cursor_x, cursor_y;               // Cursor position
    uint32_t cursor_address;                   // Cursor pattern address
    uint32_t display_start;                    // Display start address
    uint32_t disp_width;                       // Display width in pixels
    uint8_t bpp;                               // Bits per pixel
    bool linear_mode;                          // Linear mode enabled
    
    // FIFO state
    uint8_t fifo_watermark;                    // FIFO watermark
    uint8_t burst_length;                      // FIFO burst length
    
    // PLL settings
    struct {
        uint8_t n, m, p;
        uint32_t freq;
    } pll[3];                                  // NVPLL, MPLL, VPLL
    
    // Memory configuration
    uint32_t vram_size;                        // VRAM size in bytes
    uint8_t vram_config;                       // VRAM configuration
    
    // Bank switching
    uint8_t bank_read;                         // Current read bank
    uint8_t bank_write;                        // Current write bank
    
    // VGA compatibility state
    bool vga_compat_mode;                      // VGA compatibility mode
    
    // PCI configuration
    uint8_t pci_config[256];
} NV3State;

static NV3State nv3 = { 0 };

// Calculate PLL frequency from N, M, P values
uint32_t NV3_CalcPLLFreq(uint8_t n, uint8_t m, uint8_t p) {
    if (n == 0 || m == 0) return 0;
    
    // NV3 clock formula: clock = (ref_freq * m) / (n * 2^p)
    return (NV3_PLL_REF_FREQ * m) / (n * (1 << p));
}

// Set PLL frequency
void NV3_SetPLL(int which, uint32_t target_freq) {
    uint8_t n = 1, m = 1, p = 0;
    uint8_t best_n = 1, best_m = 1, best_p = 0;
    uint32_t best_freq = 0;
    uint32_t best_diff = ~0U;
    
    // Find best PLL parameters for target frequency
    for (p = 0; p <= 4; p++) {         // P is 4 bits
        for (n = 1; n <= 255; n++) {   // N is 8 bits
            // Calculate M for this N, P combination
            m = (target_freq * n * (1 << p) + NV3_PLL_REF_FREQ/2) / NV3_PLL_REF_FREQ;
            if (m < 1) m = 1;
            if (m > 255) m = 255;      // M is 8 bits
            
            // Calculate resulting frequency
            uint32_t freq = NV3_CalcPLLFreq(n, m, p);
            uint32_t diff = (freq > target_freq) ? (freq - target_freq) : (target_freq - freq);
            
            if (diff < best_diff) {
                best_diff = diff;
                best_n = n;
                best_m = m;
                best_p = p;
                best_freq = freq;
            }
        }
    }
    
    // Store best values
    if (which >= 0 && which < 3) {
        nv3.pll[which].n = best_n;
        nv3.pll[which].m = best_m;
        nv3.pll[which].p = best_p;
        nv3.pll[which].freq = best_freq;
    }
}

// Update color mode based on register settings
void NV3_UpdateColorMode(void) {
    uint8_t pixel_format = nv3.crtc_reg[NV3_CRTC_PIXEL] & 0x03;
    
    switch (pixel_format) {
        case NV3_PIXEL_FORMAT_32BPP:
            nv3.bpp = 32;
            vga.s3.xga_color_mode = M_LIN32;
            break;
        case NV3_PIXEL_FORMAT_16BPP:
            nv3.bpp = 16;
            vga.s3.xga_color_mode = M_LIN16;
            break;
        case NV3_PIXEL_FORMAT_8BPP:
            nv3.bpp = 8;
            vga.s3.xga_color_mode = M_LIN8;
            break;
        case NV3_PIXEL_FORMAT_VGA:
        default:
            nv3.bpp = 8;
            vga.s3.xga_color_mode = M_LIN8;
            break;
    }
}

// Update display start address based on register values
void NV3_UpdateDisplayStart(void) {
    // Start with basic address from standard VGA registers
    uint32_t start_addr = (nv3.crtc_reg[NV3_CRTC_START_ADDR_HIGH] << 8) | 
                           nv3.crtc_reg[NV3_CRTC_START_ADDR_LOW];
    
    // Add extended bits from REPAINT0 register
    start_addr |= ((nv3.crtc_reg[NV3_CRTC_REPAINT0] & 0x1F) << 16);
    
    // Add extended bits from graphic cursor register if used for this purpose
    if (nv3.crtc_reg[NV3_CRTC_GRCURSOR0] & 0x3F) {
        start_addr |= ((nv3.crtc_reg[NV3_CRTC_GRCURSOR0] & 0x3F) << 21);
    }
    
    nv3.display_start = start_addr;
    vga.config.display_start = start_addr;
}

// Update scan line offset (pitch) based on register values
void NV3_UpdateScanLineOffset(void) {
    // Start with standard VGA offset
    uint16_t offset = nv3.crtc_reg[NV3_CRTC_OFFSET];
    
    // Add extended bits from REPAINT0 and EXTRA registers
    offset |= ((nv3.crtc_reg[NV3_CRTC_REPAINT0] & 0xE0) >> 5) << 8;  // Bits 8-10
    offset |= ((nv3.crtc_reg[NV3_CRTC_EXTRA] & 0x20) >> 5) << 11;    // Bit 11
    
    vga.config.scan_len = offset;
    VGA_CheckScanLength();
}

// Update linear framebuffer address
void NV3_UpdateLFBAddress(uint32_t addr) {
    nv3.linear_fb_addr = addr;
    // Additional code needed to actually map the LFB in the memory space
    LOG(LOG_VGAMISC, LOG_DEBUG)("NV3: Linear framebuffer address set to %08x", nv3.linear_fb_addr);
}

// CRTC register write handler
void SVGA_NV3_WriteCRTC(Bitu reg, Bitu val, Bitu iolen) {
    if (reg > NV3_CRTC_MAX_REG) {
        LOG(LOG_VGAMISC, LOG_NORMAL)("NV3:CRTC:Write to illegal index %04x", (int)reg);
        return;
    }
    
    // Store the register value
    nv3.crtc_reg[reg] = (uint8_t)val;
    
    // Handle special register cases
    switch (reg) {
    case NV3_CRTC_HORIZ_TOTAL:
    case NV3_CRTC_HORIZ_DISPLAY_END:
    case NV3_CRTC_HORIZ_BLANK_START:
    case NV3_CRTC_HORIZ_BLANK_END:
    case NV3_CRTC_HORIZ_SYNC_START:
    case NV3_CRTC_HORIZ_SYNC_END:
    case NV3_CRTC_VERT_TOTAL:
    case NV3_CRTC_OVERFLOW:
    case NV3_CRTC_VERT_RETRACE_START:
    case NV3_CRTC_VERT_RETRACE_END:
    case NV3_CRTC_VERT_DISPLAY_END:
    case NV3_CRTC_VERT_BLANK_START:
    case NV3_CRTC_VERT_BLANK_END:
    case NV3_CRTC_HORIZ_EXTRA:        // Extended horizontal timing bits
    case NV3_CRTC_EXTRA:              // Extended vertical timing bits
        // Standard VGA and extended timing registers
        // Writing to these may trigger a mode change
        VGA_StartResize();
        break;
        
    case NV3_CRTC_START_ADDR_HIGH:
    case NV3_CRTC_START_ADDR_LOW:
    case NV3_CRTC_REPAINT0:           // Extended display address bits
    case NV3_CRTC_GRCURSOR0:          // More extended display address bits
        // Update display start address
        NV3_UpdateDisplayStart();
        break;
        
    case NV3_CRTC_OFFSET:             // Scan line offset (pitch)
    case NV3_CRTC_EXTRA:              // Extra offset bit
        // Update scan line offset
        NV3_UpdateScanLineOffset();
        break;
        
    case NV3_CRTC_REPAINT1:           // Various flags
        // Check if sync polarity changed
        if ((val & (NV3_REPAINT1_HSYNC_DISABLE | NV3_REPAINT1_VSYNC_DISABLE)) !=
            (nv3.crtc_reg[NV3_CRTC_REPAINT1] & (NV3_REPAINT1_HSYNC_DISABLE | NV3_REPAINT1_VSYNC_DISABLE))) {
            VGA_StartResize();
        }
        break;
        
    case NV3_CRTC_FIFO_CONTROL:       // FIFO burst length
        nv3.burst_length = val & 0x07;
        break;
        
    case NV3_CRTC_FIFO:               // FIFO watermark
        nv3.fifo_watermark = val & NV3_FIFO_WATERMARK_MASK;
        if (val & NV3_FIFO_RESET) {
            // Reset the FIFO
            LOG(LOG_VGAMISC, LOG_DEBUG)("NV3:CRTC: FIFO Reset");
        }
        break;
        
    case NV3_CRTC_PIXEL:              // Pixel format register
        {
            uint8_t old_format = nv3.crtc_reg[NV3_CRTC_PIXEL] & 0x03;
            uint8_t new_format = val & 0x03;
            if (old_format != new_format) {
                NV3_UpdateColorMode();
                VGA_DetermineMode();
            }
        }
        break;
        
    case NV3_CRTC_GRCURSOR1:          // Graphic cursor control
        nv3.hardware_cursor_active = (val & 0x01) != 0;
        VGA_ActivateHardwareCursor();
        break;
        
    default:
        LOG(LOG_VGAMISC, LOG_DEBUG)("NV3:CRTC:Write register %02Xh value %02Xh", (int)reg, (int)val);
        break;
    }
}

// CRTC register read handler
Bitu SVGA_NV3_ReadCRTC(Bitu reg, Bitu iolen) {
    if (reg > NV3_CRTC_MAX_REG) {
        LOG(LOG_VGAMISC, LOG_NORMAL)("NV3:CRTC:Read from illegal index %04x", (int)reg);
        return 0x00;
    }
    
    switch (reg) {
    case NV3_CRTC_REVISION_ID:
        // Return NV3 ID
        return 0x03;  // NV3 revision
        
    case NV3_CRTC_GRCURSOR1:
        // Return hardware cursor status
        return (nv3.hardware_cursor_active ? 0x01 : 0x00) | 
               ((nv3.cursor_address & 0xF800) >> 11);
        
    case NV3_CRTC_GRCURSOR0:
        // Return high bits of cursor address
        return (nv3.cursor_address >> 16) & 0x3F;
        
    case NV3_CRTC_PIXEL:
        // Return pixel format
        return nv3.crtc_reg[NV3_CRTC_PIXEL];
        
    case NV3_CRTC_FIFO:
        // Return FIFO watermark
        return nv3.fifo_watermark;
        
    case NV3_CRTC_FIFO_CONTROL:
        // Return FIFO control bits
        return nv3.burst_length;
        
    case NV3_CRTC_REPAINT0:
        // Return extended offset and start address bits
        return ((nv3.display_start >> 16) & 0x1F) | 
               ((vga.config.scan_len >> 8) & 0x07) << 5;
        
    case NV3_CRTC_REPAINT1:
        // Return various control flags
        return nv3.crtc_reg[NV3_CRTC_REPAINT1];
        
    default:
        // Return the stored register value
        return nv3.crtc_reg[reg];
    }
}

// Sequencer register write handler
void SVGA_NV3_WriteSEQ(Bitu reg, Bitu val, Bitu iolen) {
    // Handle special NV3 sequencer registers
    switch (reg) {
    case 0x08:  // PLL Unlock register (if used by NV3)
        break;
        
    case 0x10:  // NVPLL low bits
        nv3.pll[0].n = val & NV3_PLL_N_MASK;
        break;
        
    case 0x11:  // NVPLL high bits
        nv3.pll[0].m = val & NV3_PLL_M_MASK;
        break;
        
    case 0x12:  // MPLL low bits
        nv3.pll[1].n = val & NV3_PLL_N_MASK;
        break;
        
    case 0x13:  // MPLL high bits
        nv3.pll[1].m = val & NV3_PLL_M_MASK;
        break;
        
    case 0x14:  // VPLL low bits
        nv3.pll[2].n = val & NV3_PLL_N_MASK;
        break;
        
    case 0x15:  // VPLL high bits
        nv3.pll[2].m = val & NV3_PLL_M_MASK;
        // Update PLL frequency and refresh display
        nv3.pll[2].freq = NV3_CalcPLLFreq(nv3.pll[2].n, nv3.pll[2].m, nv3.pll[2].p);
        VGA_StartResize();
        break;
        
    default:
        // Let VGA handle standard registers
        LOG(LOG_VGAMISC, LOG_DEBUG)("NV3:SEQ:Write register %02Xh value %02Xh", (int)reg, (int)val);
        break;
    }
}

// Sequencer register read handler
Bitu SVGA_NV3_ReadSEQ(Bitu reg, Bitu iolen) {
    // Handle special NV3 sequencer registers
    switch (reg) {
    case 0x08:  // PLL Unlock
        return 0x06;  // Unlock value
        
    case 0x10:  // NVPLL low bits
        return nv3.pll[0].n;
        
    case 0x11:  // NVPLL high bits
        return nv3.pll[0].m;
        
    case 0x12:  // MPLL low bits
        return nv3.pll[1].n;
        
    case 0x13:  // MPLL high bits
        return nv3.pll[1].m;
        
    case 0x14:  // VPLL low bits
        return nv3.pll[2].n;
        
    case 0x15:  // VPLL high bits
        return nv3.pll[2].m;
        
    default:
        // Let VGA handle standard registers
        LOG(LOG_VGAMISC, LOG_DEBUG)("NV3:SEQ:Read register %02Xh", (int)reg);
        return 0x00;
    }
}

// Get current pixel clock frequency
Bitu SVGA_NV3_GetClock(void) {
    Bitu clock = (vga.misc_output >> 2) & 3;
    if (clock == 0)
        clock = 25175000;  // Standard VGA clock 1
    else if (clock == 1)
        clock = 28322000;  // Standard VGA clock 2
    else if (clock == 2 || clock == 3) {
        // Use VPLL for clocks 2 and 3
        clock = nv3.pll[2].freq;
        if (clock == 0) {
            // Default to standard frequencies if PLL not set
            clock = (clock == 2) ? 40000000 : 36000000;
        }
    }
    
    return clock;
}

// Check if hardware cursor is active
bool SVGA_NV3_HWCursorActive(void) {
    return nv3.hardware_cursor_active;
}

// Check if a video mode is supported by the card
bool SVGA_NV3_AcceptsMode(Bitu mode) {
    // Check if mode fits in VRAM
    return VideoModeMemSize(mode) < nv3.vram_size;
}

// Determine the current video mode based on registers
void VGA_DetermineMode_NV3(void) {
    // Check if the pixel format register indicates a non-VGA mode
    if ((nv3.crtc_reg[NV3_CRTC_PIXEL] & 0x03) != NV3_PIXEL_FORMAT_VGA) {
        // Use linear framebuffer mode based on pixel format
        switch (nv3.crtc_reg[NV3_CRTC_PIXEL] & 0x03) {
            case NV3_PIXEL_FORMAT_32BPP:
                VGA_SetMode(M_LIN32);
                break;
            case NV3_PIXEL_FORMAT_16BPP:
                VGA_SetMode(M_LIN16);
                break;
            case NV3_PIXEL_FORMAT_8BPP:
                VGA_SetMode(M_LIN8);
                break;
            default:
                // Fallback to standard VGA mode detection
                goto vga_detection;
        }
        return;
    }

vga_detection:
    // Standard VGA mode detection
    if (vga.attr.mode_control & 1) {
        // Graphics mode
        if (vga.gfx.mode & 0x40) {
            // 256-color mode
            VGA_SetMode(M_VGA);
        } else if (vga.gfx.mode & 0x20) {
            // 4-color CGA mode
            VGA_SetMode(M_CGA4);
        } else if ((vga.gfx.miscellaneous & 0x0c) == 0x0c) {
            // CGA 2-color mode
            VGA_SetMode(M_CGA2);
        } else {
            // EGA/VGA mode (16-color)
            VGA_SetMode(M_EGA);
        }
    } else {
        // Text mode
        VGA_SetMode(M_TEXT);
    }
}

// Set pixel clock
void SetClock_NV3(Bitu which, Bitu target) {
    // Set target frequency for the specified PLL
    if (which < 3) {
        NV3_SetPLL(which, target);
        
        // Update display if this is the video PLL
        if (which == 2) {  // VPLL
            VGA_StartResize();
        }
    }
}

// Memory bank read/write handlers
void SVGA_NV3_SetBank(Bitu bankNo) {
    nv3.bank_read = bankNo;
    nv3.bank_write = bankNo;
    VGA_SetupHandlers();
}

// PCI device implementation class
class PCI_NV3Device : public PCI_Device {
private:
    static const uint16_t vendor=0x12d2;        // SGS/NVIDIA
public:
    static uint16_t GetDevice(void) {
        switch (nvCard) {
            case NV_Riva128:
                return 0x0018;
            default:
                break;
        };

        return 0x0018;
    }
    PCI_NV3Device() : PCI_Device(vendor,GetDevice()) {
        // Initialize NV3 PCI configuration
        
        // Revision ID
        config[0x08] = 0x10;         // Revision ID
        
        // Class codes
        config[0x09] = 0x00;         // Interface
        config[0x0a] = 0x00;         // Subclass type (VGA compatible)
        config[0x0b] = 0x03;         // Class type (Display controller)

        config[0x0d] = 0x20;
        
        // Header type
        config[0x0e] = 0x00;         // Header type (single function)
        
        // Default interrupt settings
        config[0x3c] = 0x0B;         // IRQ 10
        config[0x3d] = 0x01;         // INTA#

        config[0x3e] = 0x03;
        config[0x3f] = 0x01;
        
        // Command and status registers
        config[0x04] = 0x07;         // Command (I/O and memory access enabled, palette snooping)
        config[0x05] = 0x00;
        config[0x06] = 0xA0;         // Status (medium timing)
        config[0x07] = 0x02;         // Status (fast back-to-back)
        
        // Setup which bits in the command register can be modified
        host_writew(config_writemask+0x04, 0x0037);  // Allow changing mem/io enable and VGA palette snoop
        
        // BAR0: Memory resource - 16MB aligned
        host_writed(config_writemask+0x10, 0xFF000000);  // 16MB aligned
        host_writed(config+0x10, (((uint32_t)NV3_LFB_BASE) & 0xfffffff0) | 0x8); // Memory resource, prefetchable
        
        // BAR1: LFB Memory resource - 16MB aligned
        host_writed(config_writemask+0x14, 0xFF000000);
        host_writed(config+0x14, 0x00000000);
        
        // Save initial LFB address
        nv3.linear_fb_addr = host_readd(config+0x10) & 0xFFFFFFF0;
        nv3.mmio_base_addr = host_readd(config+0x10) & 0xFFFFFFF0;
        
        LOG(LOG_MISC, LOG_DEBUG)("PCI: NVIDIA RIVA 128 initialized with LFB at %08x", nv3.linear_fb_addr);
    }

    void config_write(uint8_t regnum, Bitu iolen, uint32_t value) override {
        if (iolen == 1) {
            const unsigned char mask = config_writemask[regnum];
            const unsigned char nmask = ~mask;
            
            // Apply write mask
            config[regnum] = ((unsigned char)value & mask) + (config[regnum] & nmask);
            
            // Handle specific registers
            switch (regnum) {
                case 0x10: case 0x11: case 0x12: case 0x13:
                    // Update memory mapping when BAR0 changes
                    // Call our function to update LFB address
                    NV3_UpdateLFBAddress(host_readd(config+0x10) & 0xFFFFFFF0);
                    break;
                    
                default:
                    break;
            }
        }
        else {
            PCI_Device::config_write(regnum, iolen, value);
        }
    }

    uint32_t config_read(uint8_t regnum, Bitu iolen) override {
        return PCI_Device::config_read(regnum, iolen);
    }
};

static PCI_Device *nv3_pci = NULL;

// Add NV3 device to PCI bus
void PCI_AddNV3Device(void) {
    if (!pcibus_enable) return;
    
    if (nv3_pci == NULL) {
        LOG(LOG_MISC, LOG_DEBUG)("PCI: Adding NV3 RIVA 128 device");
        if ((nv3_pci = new PCI_NV3Device()) == NULL)
            return;
        
        // Register with PCI bus - try to get device 0 on bus 0 if possible
        RegisterPCIDevice(nv3_pci, 0, 0);
    }
}

// Remove NV3 device from PCI bus
void PCI_RemoveNV3Device(void) {
    if (nv3_pci != NULL) {
        UnregisterPCIDevice(nv3_pci);
        delete nv3_pci;
        nv3_pci = NULL;
    }
}

// Setup NV3 SVGA card
void SVGA_Setup_NV3RIVA() {
    // Set up RIVA 128 SVGA capabilities
    svga.write_p3d5 = &SVGA_NV3_WriteCRTC;
    svga.read_p3d5 = &SVGA_NV3_ReadCRTC;
    svga.write_p3c5 = &SVGA_NV3_WriteSEQ;
    svga.read_p3c5 = &SVGA_NV3_ReadSEQ;
    
    // No specialized Attribute Controller functions
    svga.write_p3c0 = NULL;
    svga.read_p3c1 = NULL;
    
    // Set up function handlers
    svga.set_video_mode = NULL;             // Use standard implementation
    svga.determine_mode = &VGA_DetermineMode_NV3;
    svga.set_clock = &SetClock_NV3;
    svga.get_clock = &SVGA_NV3_GetClock;
    svga.hardware_cursor_active = &SVGA_NV3_HWCursorActive;
    svga.accepts_mode = &SVGA_NV3_AcceptsMode;
    
    // Allocate 4MB of VRAM for RIVA 128
    if (vga.mem.memsize == 0)
        vga.mem.memsize = vga.mem.memsize_original = 4*1024*1024;
    
    nv3.vram_size = vga.mem.memsize;
    
    // Initialize NV3 state
    memset(nv3.crtc_reg, 0, sizeof(nv3.crtc_reg));
    nv3.hardware_cursor_active = false;
    nv3.vram_config = 0x1A;                 // 4MB configuration
    nv3.linear_mode = true;
    nv3.display_start = 0;
    nv3.cursor_address = 0x1000;
    nv3.cursor_x = 0;
    nv3.cursor_y = 0;
    nv3.bpp = 8;                            // Default to 8bpp
    
    // Initialize FIFO settings
    nv3.fifo_watermark = 0x20;              // Default watermark
    nv3.burst_length = NV3_FIFO_CONTROL_BURST_LENGTH_32; // Default burst length
    
    // Set initial register values
    nv3.crtc_reg[NV3_CRTC_REPAINT1] = 0x00; // Default REPAINT1 (all enabled)
    nv3.crtc_reg[NV3_CRTC_PIXEL] = NV3_PIXEL_FORMAT_VGA; // Default to VGA mode
    
    // Set initial PLL values
    nv3.pll[0].n = 8;   // NVPLL (for 3D acceleration)
    nv3.pll[0].m = 71;
    nv3.pll[0].p = 0;
    nv3.pll[0].freq = NV3_CalcPLLFreq(8, 71, 0);
    
    nv3.pll[1].n = 13;  // MPLL (memory clock)
    nv3.pll[1].m = 112;
    nv3.pll[1].p = 0;
    nv3.pll[1].freq = NV3_CalcPLLFreq(13, 112, 0);
    
    nv3.pll[2].n = 13;  // VPLL (video clock)
    nv3.pll[2].m = 112;
    nv3.pll[2].p = 1;
    nv3.pll[2].freq = NV3_CalcPLLFreq(13, 112, 1);
    
    // Register the card in the PCI bus
    PCI_AddNV3Device();
    
    LOG(LOG_MISC, LOG_NORMAL)("SVGA: NV3 RIVA 128 initialized with %dMB VRAM", nv3.vram_size / (1024*1024));
}

// Save state support
void POD_Save_VGA_NV3(std::ostream& stream) {
    // Save NV3-specific state
    WRITE_POD(&nv3, nv3);
}

void POD_Load_VGA_NV3(std::istream& stream) {
    // Load NV3-specific state
    READ_POD(&nv3, nv3);
}