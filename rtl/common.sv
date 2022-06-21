package common;
  parameter FXP_MAX_WIDTH             = 64;
  parameter CORDIC_N_STAGES           = 19;
  parameter CORDIC_WORD_WIDTH         = 20;
  parameter CORDIC_N_STEPS            = 16;
  parameter JACOBI_OUTPUT_WORD_WIDTH  = 20; //Q(1.4.15)
  parameter JACOBI_INPUT_WORD_WIDTH   = 16; //Q(1.0.15)
  parameter JACOBI_N                  = 8;  //matrix size
  parameter JACOBI_N_INPUT_DATA       = 36;
  parameter JACOBI_LOG2_N_INPUT_DATA  = 6;
  parameter JACOBI_MEM_SIZE           = 100;
  parameter JACOBI_ADDR_WIDTH         = 7;
  parameter JACOBI_LOG2_N             = 3;
  parameter JACOBI_V_OFFSET           = 36;
  parameter JACOBI_N_PAIRS            = 4;
  parameter JACOBI_LOG2_N_PAIRS       = 2;
  parameter JACOBI_PAIR               = 2;
  parameter JACOBI_LOG2_PAIR          = 1;
  parameter JACOBI_PI                 = 102944;
  parameter JACOBI_MATRIX_VALUES_IN_ONE_PAIR = 15;
  parameter JACOBI_LOG2_MATRIX_VALUES_IN_ONE_PAIR = 4;
  parameter JACOBI_N_ROUNDS           = 28;
  
  function [FXP_MAX_WIDTH-1:0] fxp_round;
    input signed [FXP_MAX_WIDTH-1:0] in_dat;
    input integer N;
    reg signed [FXP_MAX_WIDTH-1:0] shifted_n_minus_1;

    begin
      shifted_n_minus_1 = (in_dat >>> N-1);
      fxp_round = (in_dat + (1 <<< N-1)) >>> N;
      if ((shifted_n_minus_1 <<< N-1) == in_dat && shifted_n_minus_1[0] == 1) begin //Case with 0.5
        if (fxp_round[0] == 1) begin //If output is odd round to even
          fxp_round = fxp_round - 1;
        end
      end
    end
  endfunction
endpackage