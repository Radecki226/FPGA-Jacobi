
#ifndef _JACOBI_H_
#define _JACOBI_H_
#define N_INPUT_DATA 36
#define N_OUTPUT_DATA 100

#include "xstatus.h"


// Driver user functions
int jacobi_calc( s32* input_matrix, s32* output_buffer);
int init_jacobi();
void reset_jacobi();

// Lower level driver function
int send_buffer(s32* buf);
int receive_buffer(s32* buf);

void print_symmetric_matrix(s32* buf);
void print_eigenvectors(s32* buf);


#endif
