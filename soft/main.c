/*
 * main.c: find_mx application
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "xil_printf.h"
#include "jacobi.h"
#include "platform.h"


s32 input_matrix[N_INPUT_DATA] = {
		0x00005E74,
		0xFFFFEBB6,
		0x00003158,
		0x00003B4E,
		0x00004295,
		0x000001CC,
		0x00004EEB,
		0x00001D45,
		0xFFFFDAC8,
		0x00004C20,
		0x00001E35,
		0x00000EC5,
		0xFFFFDAAE,
		0x00003EA7,
		0xFFFFD209,
		0x00006228,
		0xFFFFF40B,
		0x000017C7,
		0x0000366C,
		0x00006F0A,
		0xFFFFE8A0,
		0x000002BF,
		0xFFFFF03B,
		0xFFFFFA45,
		0x00003F9C,
		0x000010C3,
		0xFFFFAF48,
		0x00001849,
		0xFFFFDD85,
		0x00005861,
		0xFFFFB30D,
		0x00001624,
		0x00002440,
		0xFFFF9BF9,
		0x00003B35,
		0x00007A5D
};

s32 output_matrix[N_OUTPUT_DATA];


int main()
{

	// Initialize FIFOs and accelerator. Check status
	init_platform();
	if ( init_jacobi() == XST_FAILURE )
		goto error;

    print("Let's calculate eigenvalues \n\r");
    print("Input matrix:\n\r");
    print_symmetric_matrix(input_matrix);

    jacobi_calc(input_matrix, output_matrix);

    /*for (int i = 0; i < N_OUTPUT_DATA; i++) {
    	xil_printf("Data %d = %d\n\r", i, output_matrix[i]);
    }*/

    xil_printf("Eigenvalues matrix: \n\r");
    print_symmetric_matrix(output_matrix);
    xil_printf("Eigenvectors matrix: \n\r");
    print_eigenvectors(output_matrix + 36);


	

error:
	reset_jacobi();
    cleanup_platform();
    while(1);
}
