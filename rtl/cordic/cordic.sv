//Q(1.4.15)
import common::*;
module cordic #(
  parameter MODE = "rotation"
)
(
  input clk,
  input rst,

  input [CORDIC_WORD_WIDTH-1:0] x_i,
  input [CORDIC_WORD_WIDTH-1:0] y_i,
  input [CORDIC_WORD_WIDTH-1:0] z_i,
  input                   vld_i,

  output [CORDIC_WORD_WIDTH-1:0] x_o,
  output [CORDIC_WORD_WIDTH-1:0] y_o,
  output [CORDIC_WORD_WIDTH-1:0] z_o,
  output                   vld_o
);

  reg signed [CORDIC_WORD_WIDTH-1:0] atan [0:CORDIC_N_STEPS-1] = {'h06488, 'h03B59, 'h01F5B, 'h00FEB, 'h007FD,
                                                                  'h00400, 'h00200, 'h00100, 'h00080, 'h00040, 
                                                                  'h00020, 'h00010, 'h00008, 'h00004, 'h00002, 'h00001};
  reg [CORDIC_WORD_WIDTH-1:0] x_vec_r   [CORDIC_N_STAGES:0];
  reg [CORDIC_WORD_WIDTH-1:0] y_vec_r   [CORDIC_N_STAGES:0];
  reg [CORDIC_WORD_WIDTH-1:0] z_vec_r   [CORDIC_N_STAGES:0];
  reg                         vld_vec_r [CORDIC_N_STAGES:0]; 

  assign x_vec_r[0]   = x_i;
  assign y_vec_r[0]   = y_i;
  assign z_vec_r[0]   = z_i;
  assign vld_vec_r[0] = vld_i;

  //steps 0-15
  genvar i;
  generate
    for (i = 0; i < CORDIC_N_STEPS; i++) begin
      cordic_step #(MODE, CORDIC_WORD_WIDTH, i) step (
        clk,
        rst, 
        x_vec_r[i],
        y_vec_r[i],
        z_vec_r[i],
        atan[i],
        vld_vec_r[i],
        x_vec_r[i+1],
        y_vec_r[i+1],
        z_vec_r[i+1],
        vld_vec_r[i+1]
      );
    end
  endgenerate

  //steps 16-18
  cordic_mult #(CORDIC_WORD_WIDTH) x_mult (
    clk, 
    rst, 
    x_vec_r[CORDIC_N_STEPS],
    x_vec_r[CORDIC_N_STAGES]
  );
  cordic_mult #(CORDIC_WORD_WIDTH) y_mult (
    clk, 
    rst, 
    y_vec_r[CORDIC_N_STEPS],
    y_vec_r[CORDIC_N_STAGES]
  );

  //Placeholder for z and valid wile x,y are calculated
  always_ff @(posedge clk) begin
    z_vec_r   [CORDIC_N_STAGES:CORDIC_N_STEPS+1] <= z_vec_r[CORDIC_N_STAGES-1:CORDIC_N_STEPS];
    vld_vec_r [CORDIC_N_STAGES:CORDIC_N_STEPS+1] <= vld_vec_r[CORDIC_N_STAGES-1:CORDIC_N_STEPS];

    if (rst == 1) begin
      vld_vec_r [CORDIC_N_STAGES:CORDIC_N_STEPS+1] <= {0,0,0};
    end
  end

  assign x_o = x_vec_r[CORDIC_N_STAGES];
  assign y_o = y_vec_r[CORDIC_N_STAGES];
  assign z_o = z_vec_r[CORDIC_N_STAGES];
  assign vld_o = vld_vec_r[CORDIC_N_STAGES];
    
endmodule