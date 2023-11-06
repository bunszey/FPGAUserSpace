// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XINVERTER_H
#define XINVERTER_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xinverter_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Axi_cpu_BaseAddress;
} XInverter_Config;
#endif

typedef struct {
    u64 Axi_cpu_BaseAddress;
    u32 IsReady;
} XInverter;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XInverter_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XInverter_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XInverter_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XInverter_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XInverter_Initialize(XInverter *InstancePtr, u16 DeviceId);
XInverter_Config* XInverter_LookupConfig(u16 DeviceId);
int XInverter_CfgInitialize(XInverter *InstancePtr, XInverter_Config *ConfigPtr);
#else
int XInverter_Initialize(XInverter *InstancePtr, const char* InstanceName);
int XInverter_Release(XInverter *InstancePtr);
#endif

void XInverter_Start(XInverter *InstancePtr);
u32 XInverter_IsDone(XInverter *InstancePtr);
u32 XInverter_IsIdle(XInverter *InstancePtr);
u32 XInverter_IsReady(XInverter *InstancePtr);
void XInverter_EnableAutoRestart(XInverter *InstancePtr);
void XInverter_DisableAutoRestart(XInverter *InstancePtr);


void XInverter_InterruptGlobalEnable(XInverter *InstancePtr);
void XInverter_InterruptGlobalDisable(XInverter *InstancePtr);
void XInverter_InterruptEnable(XInverter *InstancePtr, u32 Mask);
void XInverter_InterruptDisable(XInverter *InstancePtr, u32 Mask);
void XInverter_InterruptClear(XInverter *InstancePtr, u32 Mask);
u32 XInverter_InterruptGetEnabled(XInverter *InstancePtr);
u32 XInverter_InterruptGetStatus(XInverter *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
