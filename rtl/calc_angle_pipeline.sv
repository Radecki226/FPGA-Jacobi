import common::*;
module calc_angle_pipeline(
  input clk,
  input rst,

  input signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] x_i,
  input signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] y_i,
  input                                       vld_i,


  output signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] angle_o,
  output                                       vld_o
);
  

  reg [JACOBI_OUTPUT_WORD_WIDTH-1:0]        x_r;
  reg [JACOBI_OUTPUT_WORD_WIDTH-1:0]        y_r;
  reg signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] pi_to_add_r;
  reg                                       in_vld_r;
  reg signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] cordic_z;
  reg signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] angle_r;
  reg                                       cordic_vld;
  reg                                       out_vld_r;

  reg signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] pi_shift_reg_r [CORDIC_N_STAGES-1:0];

  always_ff @(posedge clk) begin : input_stage_p
    in_vld_r <= vld_i;
    if (x_i > 0) begin
      y_r <= y_i;
      x_r <= x_i;
      pi_to_add_r <= 0;
    end else begin
      y_r <= -y_i;
      x_r <= -x_i;
      if (y_i > 0) begin
        pi_to_add_r <= JACOBI_PI;
      end else begin
        pi_to_add_r <= -JACOBI_PI;
      end
    end
  end

  cordic #(.MODE("vectoring")) vectoring_cordic_i (
    .clk(clk),
    .rst(rst),

    .x_i(x_r),
    .y_i(y_r),
    .z_i(0),
    .vld_i(in_vld_r),

    .z_o(cordic_z),
    .vld_o(cordic_vld)
  );
  
  // Shift register in order to add pi_to_add value to pipeline
  always_ff @(posedge clk) begin
    pi_shift_reg_r[CORDIC_N_STAGES-1:1] <= pi_shift_reg_r[CORDIC_N_STAGES-2:0];
    pi_shift_reg_r[0] <= pi_to_add_r;
  end
  
  //output stage
  always_ff @(posedge clk) begin : add_pi_and_shift_p
    angle_r   <= fxp_round(cordic_z + pi_shift_reg_r[CORDIC_N_STAGES-1], 1);
    out_vld_r <= cordic_vld;
  end

  assign angle_o = angle_r;
  assign vld_o = out_vld_r;


endmodule