
module cordic_step #(
  parameter MODE = "rotation",
  parameter WORD_WIDTH = 20,
  parameter shift = 0
)
(
  input clk,
  input rst,

  input signed [WORD_WIDTH-1:0] x_i,
  input signed [WORD_WIDTH-1:0] y_i,
  input signed [WORD_WIDTH-1:0] z_i,
  input signed [WORD_WIDTH-1:0] atan_i,
  input                   vld_i,

  output signed [WORD_WIDTH-1:0] x_o,
  output signed [WORD_WIDTH-1:0] y_o,
  output signed [WORD_WIDTH-1:0] z_o,
  output                   vld_o

);
  
  reg [WORD_WIDTH-1:0] x_r;
  reg [WORD_WIDTH-1:0] y_r;
  reg [WORD_WIDTH-1:0] z_r;
  reg                   vld_r;

generate
  if (MODE == "rotation") begin
    always_ff @(posedge clk) begin
      if (z_i >= 0) begin
        x_r <= x_i - fxp_round(y_i, shift);
        y_r <= y_i + fxp_round(x_i, shift);
        z_r <= z_i - atan_i;
      end else begin
        x_r <= x_i + fxp_round(y_i, shift);
        y_r <= y_i - fxp_round(x_i, shift);
        z_r <= z_i + atan_i; 
      end
    end

  end else if (MODE == "vectoring") begin
    always_ff @(posedge clk) begin
      if (y_i <= 0) begin
        x_r <= x_i - fxp_round(y_i, shift);
        y_r <= y_i + fxp_round(x_i, shift);
        z_r <= z_i - atan_i;
      end else begin
        x_r <= x_i + fxp_round(y_i, shift);
        y_r <= y_i - fxp_round(x_i, shift);
        z_r <= z_i + atan_i; 
      end
    end

  end
endgenerate
  always_ff @(posedge clk) begin
    vld_r <= vld_i;
    if (rst == 1) begin
      vld_r <= 0;
    end
  end

  assign x_o = x_r;
  assign y_o = y_r;
  assign z_o = z_r;
  assign vld_o = vld_r;

endmodule


  
