// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xinverter.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XInverter_CfgInitialize(XInverter *InstancePtr, XInverter_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Axi_cpu_BaseAddress = ConfigPtr->Axi_cpu_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XInverter_Start(XInverter *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_AP_CTRL) & 0x80;
    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_AP_CTRL, Data | 0x01);
}

u32 XInverter_IsDone(XInverter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XInverter_IsIdle(XInverter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XInverter_IsReady(XInverter *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XInverter_EnableAutoRestart(XInverter *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_AP_CTRL, 0x80);
}

void XInverter_DisableAutoRestart(XInverter *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_AP_CTRL, 0);
}

void XInverter_InterruptGlobalEnable(XInverter *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_GIE, 1);
}

void XInverter_InterruptGlobalDisable(XInverter *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_GIE, 0);
}

void XInverter_InterruptEnable(XInverter *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_IER);
    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_IER, Register | Mask);
}

void XInverter_InterruptDisable(XInverter *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_IER);
    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_IER, Register & (~Mask));
}

void XInverter_InterruptClear(XInverter *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XInverter_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_ISR, Mask);
}

u32 XInverter_InterruptGetEnabled(XInverter *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_IER);
}

u32 XInverter_InterruptGetStatus(XInverter *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XInverter_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XINVERTER_AXI_CPU_ADDR_ISR);
}

