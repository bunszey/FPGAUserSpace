// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xinverter.h"

extern XInverter_Config XInverter_ConfigTable[];

XInverter_Config *XInverter_LookupConfig(u16 DeviceId) {
	XInverter_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XINVERTER_NUM_INSTANCES; Index++) {
		if (XInverter_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XInverter_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XInverter_Initialize(XInverter *InstancePtr, u16 DeviceId) {
	XInverter_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XInverter_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XInverter_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

