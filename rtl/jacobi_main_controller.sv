module jacobi_main_controller #(
  parameter N = 8,
  parameter IN_WORD_WIDTH = 16,
  parameter OUT_WORD_WIDTH = 20,
  parameter MEM_ADDR_WIDTH = 7
)  
(
  input                       clk,
  input                       rst, 

  // Input interface from microcontroller
  input  [IN_WORD_WIDTH-1:0]  in_dat_i,
  input                       in_vld_i,
  output                      in_rdy_o,

  // Output interface to microcontroller
  output [OUT_WORD_WIDTH-1:0] out_dat_o,
  output                      out_vld_o,
  input                       out_rdy_i,

  // Output data to pipelined vectoring cordic (angle calc)
  output [OUT_WORD_WIDTH-1:0] vectoring_in_dat_x_o,
  output [OUT_WORD_WIDTH-1:0] vectoring_in_dat_y_o,
  output [OUT_WORD_WIDTH-1:0] vectoring_in_dat_z_o,
  output                      vectoring_in_vld_o,

  // Input data from vectoring cordic
  input  [OUT_WORD_WIDTH-1:0] vectoring_out_dat_x_i,
  input  [OUT_WORD_WIDTH-1:0] vectoring_out_dat_y_i,
  input  [OUT_WORD_WIDTH-1:0] vectoring_out_dat_z_i,
  output                      vectoring_out_vld_i,

  //Memory interface
  output                      ram_en_a_o,
  output                      ram_we_a_o,
  output [MEM_ADDR_WIDTH-1:0] ram_addr_a_o,
  output [OUT_WORD_WIDTH-1:0] ram_din_a_o,
  input  [OUT_WORD_WIDTH-1:0] ram_dout_a_i,

  output                      ram_en_b_o,
  output                      ram_we_b_o,
  output [MEM_ADDR_WIDTH-1:0] ram_addr_b_o,
  output [OUT_WORD_WIDTH-1:0] ram_din_b_o,
  input  [OUT_WORD_WIDTH-1:0] ram_dout_b_i,

  // Output data to rotation cordic
  output [OUT_WORD_WIDTH-1:0] rotation_in_dat_x_o,
  output [OUT_WORD_WIDTH-1:0] rotation_in_dat_y_o,
  output [OUT_WORD_WIDTH-1:0] rotation_in_dat_z_o,
  output                      rotation_in_vld_o,

  // Input data from rotation cordic output fifo
  input  [OUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_x_i,
  input  [OUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_y_i,
  input  [OUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_z_i,
  output                      rotation_fifo_out_vld_i
);

endmodule
