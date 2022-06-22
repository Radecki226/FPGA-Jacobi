

#include "xparameters.h"
#include "xllfifo.h"
#include "xstatus.h"
#include "jacobi.h"
#include "xil_printf.h"

XLlFifo Instance;

#define FIFO_DEVICE_ID XPAR_AXI_FIFO_MM_S_0_DEVICE_ID





int jacobi_calc( s32* input_matrix, s32* output_buffer)
{


	if( send_buffer(input_matrix) == XST_FAILURE )
		goto error;
	//Get results
    if( receive_buffer(output_buffer) == XST_FAILURE )
		goto error;
    //Return number of results in bytes
	return 1;

error:
	return 0;

}


/**
 * Initialize FIFOs and its driver
 */
int init_jacobi(){
XLlFifo_Config *Config;
int Status;


	/* Initialize the Device Configuration Interface driver */
	Config = XLlFfio_LookupConfig(FIFO_DEVICE_ID);
	if (!Config) {
		return XST_FAILURE;
	}

	Status = XLlFifo_CfgInitialize(&Instance, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/* Check for the Reset value */
	Status = XLlFifo_Status(&Instance);
	XLlFifo_IntClear(&Instance,0xffffffff);
	Status = XLlFifo_Status(&Instance);
	if(Status != 0x0) {
		return XST_FAILURE;
	}


	return XST_SUCCESS;

}

void reset_jacobi(){
	XLlFifo_RxReset(&Instance);
	XLlFifo_TxReset(&Instance);
}

/**
 * Send data to the input FIFO and accelerator
 */
int send_buffer(s32* buf){

	//Write data to the input FIFO
	XLlFifo_Write(&Instance, buf, N_INPUT_DATA*sizeof(s32));
	//Initialize data transfer
 	XLlFifo_TxSetLen(&Instance, N_INPUT_DATA*sizeof(s32));

 	// Check for Transmission completion
 	while( !(XLlFifo_IsTxDone(&Instance)) ){

 	}

 	return XST_SUCCESS;
}


/**
 * Receive date form the output FIFO
 */
int receive_buffer(s32* buf){
u32 bytes;
int Status;

	//wait for data frame ready
	while(XLlFifo_RxOccupancy(&Instance)==0);
	//get number of data in frame
	bytes = XLlFifo_RxGetLen(&Instance);
	u32 expected_bytes = N_OUTPUT_DATA * sizeof(s32);
	//Expected number of elements should be ready
	if( expected_bytes != bytes ) return XST_FAILURE;

    //Perform read operation form FIFO
	XLlFifo_Read(&Instance, buf, N_OUTPUT_DATA * sizeof(s32));

	//Check operation status
	Status = XLlFifo_IsRxDone(&Instance);
	if(Status != TRUE){
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

s32 jacobi_correct_format(s32 dat){
	s32 check = (dat >> 19) & 1;
	if (check == 0) {
		return dat;
	} else {
		return (dat | 0xfff00000);
	}
}

void print_symmetric_matrix(s32* buf){
	int row[8];
	int row_integer[8];
	int row_fractional[8];
	char sign[8];
	for (int i = 0; i < 8; i++){
		for (int j = 0; j < 8; j++){
			if (i > j) {
				row[j] = jacobi_correct_format(buf[8*j + i - j*(j+1)/2]);
			} else {
				row[j] = jacobi_correct_format(buf[8*i + j - i*(i+1)/2]);
			}
			if (row[j] >= 0) {
				sign[j] = 20;
				row_integer[j] = row[j] >> 15;
				row_fractional[j] = ((row[j]&0x7fff)*10000)/32768;
			} else {
				sign[j] = '-';
				row_integer[j] = (-row[j] >> 15);
				row_fractional[j] = (( -(row[j]) &0x7fff)*10000)/32768;
			}


		}
		/*xil_printf("[%d %d %d %d %d %d %d %d]\n\r", row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]);*/
	    xil_printf("[%c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d]\n\r", sign[0], row_integer[0], row_fractional[0],
				sign[1], row_integer[1], row_fractional[1], sign[2], row_integer[2], row_fractional[2], sign[3], row_integer[3], row_fractional[3],
				sign[4], row_integer[4], row_fractional[4], sign[5], row_integer[5], row_fractional[5], sign[6], row_integer[6], row_fractional[6],
				sign[7], row_integer[7], row_fractional[7]);
	}
}

void print_eigenvectors(s32* buf) {
	int row[8];
	int row_integer[8];
	int row_fractional[8];
	char sign[8];
	for (int i = 0; i < 8; i++){
		for (int j = 0; j < 8; j++){
			row[j] = jacobi_correct_format(buf[8*i + j]);

			if (row[j] >= 0) {
				sign[j] = 20;
				row_integer[j] = row[j] >> 15;
				row_fractional[j] = ((row[j]&0x7fff)*10000)/32768;
			} else {
				sign[j] = '-';
				row_integer[j] = (-row[j] >> 15);
				row_fractional[j] = (( -(row[j]) &0x7fff)*10000)/32768;
			}
		}
	    xil_printf("[%c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d %c%d.%04d]\n\r", sign[0], row_integer[0], row_fractional[0],
				sign[1], row_integer[1], row_fractional[1], sign[2], row_integer[2], row_fractional[2], sign[3], row_integer[3], row_fractional[3],
				sign[4], row_integer[4], row_fractional[4], sign[5], row_integer[5], row_fractional[5], sign[6], row_integer[6], row_fractional[6],
				sign[7], row_integer[7], row_fractional[7]);
	}
}

