module jacobi_top #(
  parameter N = 8,
  parameter IN_WORD_WIDTH = 16,
  parameter OUT_WORD_WIDTH = 20
)
(
  input                       clk,
  input                       rst, 

  input [IN_WORD_WIDTH-1:0]   in_dat_i,
  input                       in_vld_i,
  output                      in_rdy_o,

  output [OUT_WORD_WIDTH-1:0] out_dat_o,
  output                      out_vld_o,
  input                       out_rdy_i
);
  
  reg [OUT_WORD_WIDTH-1:0] out_dat_r;
  reg                  out_vld_r = 0;
  
  always_ff @ (posedge clk) begin
    out_dat_r[IN_WORD_WIDTH-1:0] <= in_dat_i;
    out_vld_r                    <= in_vld_i;
  end

    assign in_rdy_o = 1;
    assign out_dat_o = out_dat_r;
    assign out_vld_o = out_vld_r;
    
endmodule