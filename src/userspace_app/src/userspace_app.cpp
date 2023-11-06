#include <stdio.h>
//#include "platform.h"
//#include "xil_printf.h"
#include "xinverter.h"
//#include "xparameters.h"

#define DATA_SIZE 307200//100000//307200

int main()
{
    //init_platform();
    printf("Hello World\n\r");

    XInverter ip_inst;
    char instance_name[ 20 ];
	sprintf(instance_name, "inverter");

	int status = XInverter_Initialize(&ip_inst, instance_name);
	if (status != XST_SUCCESS) {
		printf("Error: Could not initialize the IP core.\n\r");
	} else {
		printf("Initialized the IP core.\n\r");
	}

    // Perform operations with the IP core here

    // Test the IP core functionality
    unsigned char in[DATA_SIZE];
    unsigned char in_cpy[DATA_SIZE];
    unsigned char out[DATA_SIZE];

    // Initialize input arrays
    for (int i = 0; i < DATA_SIZE; i++) {
        in[i] = i%255;
    }

    // Call the IP core function
    XInverter_Write_in_r_Bytes(&ip_inst, 0, in, DATA_SIZE);
//    XExample_IsIdle(&ip_inst);
//    XExample_Start(&ip_inst);
    XInverter_Start(&ip_inst);

    // Wait for the IP core to finish
    while (!XInverter_IsDone(&ip_inst));

    XInverter_Read_in_r_Bytes(&ip_inst, 0, in_cpy, DATA_SIZE);
    XInverter_Read_out_r_Bytes(&ip_inst, 0, out, DATA_SIZE);
    // Check the results
    int errors = 0;
    for (int i = 0; i < DATA_SIZE; i++) {
        int expected = 255 - in[i];
//        printf("\nin1_cpy[%d]: %d\n",i, in1_cpy[i]);
//        printf("in2_cpy[%d]: %d\n",i, in2_cpy[i]);
//        printf("result[%d]: %d\n",i, out[i]);
        if (out[i] != expected) {
            errors++;
        }
    }

    if (errors == 0) {
        printf("Test passed.\n");
        printf("Data in out array: %d \n", out[DATA_SIZE - 1]);
        printf("Data in in array: %d \n", in[DATA_SIZE - 1]);
        printf("Data in in_copy array: %d \n", in_cpy[DATA_SIZE - 1]);
    } else {
        printf("Test failed with %d errors.\n", errors);
    }

    // Cleanup
	XInverter_DisableAutoRestart(&ip_inst);
	XInverter_Release(&ip_inst);

    return XST_SUCCESS;
}