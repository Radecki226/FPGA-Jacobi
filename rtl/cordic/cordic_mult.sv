import common::*;
module cordic_mult #(
  parameter WORD_WIDTH = 20
)
(
  input clk,
  input rst,

  input signed [WORD_WIDTH-1:0]  dat_i,

  output signed [WORD_WIDTH-1:0] dat_o
);
  reg signed   [WORD_WIDTH-1:0] dat_0_3, dat3, dat_6_9_13, dat_1_5, dat_8_12_14, dat_plus, dat_minus, dat_o_r;
  //stage 16
  always_ff @(posedge clk) begin
    //plus sign
    dat_0_3    <= dat_i + fxp_round(dat_i, 3);
    dat_6_9_13 <= fxp_round(dat_i,6) + fxp_round(dat_i,9) + fxp_round(dat_i,13);

    //minus sign
    dat_1_5     <= fxp_round(dat_i, 1) + fxp_round(dat_i, 5);   
    dat_8_12_14 <= fxp_round(dat_i, 8) + fxp_round(dat_i, 12) + fxp_round(dat_i, 14); 
  end

  //stage 17
  always_ff @(posedge clk) begin
    //plus sign
    dat_plus  <= dat_0_3 + dat_6_9_13;
    dat_minus <= dat_1_5 + dat_8_12_14;
  end

  //stage 18
  always_ff @(posedge clk) begin
    dat_o_r <= dat_plus - dat_minus;
  end

  assign dat_o = dat_o_r;

endmodule
  