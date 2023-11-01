// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// AXI_CPU
// 0x000000 : Control signals
//            bit 0  - ap_start (Read/Write/COH)
//            bit 1  - ap_done (Read/COR)
//            bit 2  - ap_idle (Read)
//            bit 3  - ap_ready (Read)
//            bit 7  - auto_restart (Read/Write)
//            others - reserved
// 0x000004 : Global Interrupt Enable Register
//            bit 0  - Global Interrupt Enable (Read/Write)
//            others - reserved
// 0x000008 : IP Interrupt Enable Register (Read/Write)
//            bit 0  - enable ap_done interrupt (Read/Write)
//            bit 1  - enable ap_ready interrupt (Read/Write)
//            others - reserved
// 0x00000c : IP Interrupt Status Register (Read/TOW)
//            bit 0  - ap_done (COR/TOW)
//            bit 1  - ap_ready (COR/TOW)
//            others - reserved
// 0x080000 ~
// 0x0fffff : Memory 'in_r' (307200 * 8b)
//            Word n : bit [ 7: 0] - in_r[4n]
//                     bit [15: 8] - in_r[4n+1]
//                     bit [23:16] - in_r[4n+2]
//                     bit [31:24] - in_r[4n+3]
// 0x100000 ~
// 0x17ffff : Memory 'out_r' (307200 * 8b)
//            Word n : bit [ 7: 0] - out_r[4n]
//                     bit [15: 8] - out_r[4n+1]
//                     bit [23:16] - out_r[4n+2]
//                     bit [31:24] - out_r[4n+3]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XEXAMPLE_AXI_CPU_ADDR_AP_CTRL    0x000000
#define XEXAMPLE_AXI_CPU_ADDR_GIE        0x000004
#define XEXAMPLE_AXI_CPU_ADDR_IER        0x000008
#define XEXAMPLE_AXI_CPU_ADDR_ISR        0x00000c
#define XEXAMPLE_AXI_CPU_ADDR_IN_R_BASE  0x080000
#define XEXAMPLE_AXI_CPU_ADDR_IN_R_HIGH  0x0fffff
#define XEXAMPLE_AXI_CPU_WIDTH_IN_R      8
#define XEXAMPLE_AXI_CPU_DEPTH_IN_R      307200
#define XEXAMPLE_AXI_CPU_ADDR_OUT_R_BASE 0x100000
#define XEXAMPLE_AXI_CPU_ADDR_OUT_R_HIGH 0x17ffff
#define XEXAMPLE_AXI_CPU_WIDTH_OUT_R     8
#define XEXAMPLE_AXI_CPU_DEPTH_OUT_R     307200

