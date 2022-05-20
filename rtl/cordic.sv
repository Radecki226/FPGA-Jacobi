//Q(1.4.15)
`define WORD_WIDTH 20
//`FRAC_WIDTH 15
module cordic #(
  parameter MODE = "rotation"
)
(
  input clk,
  input rst,

  input [`WORD_WIDTH-1:0] x_i,
  input [`WORD_WIDTH-1:0] y_i,
  input [`WORD_WIDTH-1:0] z_i,
  input                   vld_i,

  output [`WORD_WIDTH-1:0] x_o,
  output [`WORD_WIDTH-1:0] y_o,
  output [`WORD_WIDTH-1:0] z_o,
  output                   vld_o
);

  reg [`WORD_WIDTH-1:0] x_r;
  reg [`WORD_WIDTH-1:0] y_r;
  reg [`WORD_WIDTH-1:0] z_r;
  reg                   vld_r;

  always_ff @(posedge clk) begin
    x_r   <= x_i;
    y_r   <= y_i;
    z_r   <= z_i;
    vld_r <= vld_i;
  end

  assign x_o = x_r;
  assign y_o = y_r;
  assign z_o = z_r;
  assign vld_o = vld_r;
    
endmodule