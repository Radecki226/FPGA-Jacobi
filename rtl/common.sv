package common;
  parameter FXP_MAX_WIDTH             = 64;
  parameter CORDIC_N_STAGES           = 19;
  parameter CORDIC_WORD_WIDTH         = 20;
  parameter CORDIC_N_STEPS            = 16;
  parameter JACOBI_OUTPUT_WORD_WIDTH  = 20; //Q(1.4.15)
  parameter JACOBI_INPUT_WORD_WIDTH   = 16; //Q(1.0.15)
  parameter JACOBI_N                  = 8;  //matrix size
  
  function [FXP_MAX_WIDTH-1:0] fxp_round;
    input signed [FXP_MAX_WIDTH-1:0] in_dat;
    input integer N;
    begin 
      fxp_round = (in_dat + (1 <<< N-1)) >>> N;
    end
  endfunction
endpackage