import common::*;
module jacobi_top (
  input                       clk,
  input                       rst, 

  input [JACOBI_INPUT_WORD_WIDTH-1:0] in_dat_i,
  input                       in_vld_i,
  output                      in_rdy_o,

  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] out_dat_o,
  output                      out_vld_o,
  input                       out_rdy_i
);

  /*********************
   * Signal declarations
  **********************/
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_x;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_y;
  wire                                vectoring_in_vld;


  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_out_angle;
  wire                                vectoring_out_vld;

  wire                                ram_en_a;
  wire                                ram_we_a;
  wire [JACOBI_ADDR_WIDTH-1:0]        ram_addr_a;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_a;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_dout_a;

  wire                                ram_en_b;
  wire                                ram_we_b;
  wire [JACOBI_ADDR_WIDTH-1:0]        ram_addr_b;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_b;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_dout_b;

  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_x;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_y;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_z;
  wire                                rotation_in_vld;

  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_out_dat_x;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_out_dat_y;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_out_dat_z;
  wire                                rotation_out_vld;

  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_x;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_y;
  wire [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_z;
  wire                                rotation_fifo_out_vld;

  
  /***************
   * Instantations
  ***************/
  jacobi_main_controller main_controller (
    .clk(clk),
    .rst(rst),
    
    .in_dat_i(in_dat_i),
    .in_vld_i(in_vld_i),
    .in_rdy_o(in_rdy_o),

    .out_dat_o(out_dat_o),
    .out_vld_o(out_vld_o),
    .out_rdy_i(out_rdy_i),

    .vectoring_in_dat_x_o(vectoring_in_dat_x),
    .vectoring_in_dat_y_o(vectoring_in_dat_y),
    .vectoring_in_vld_o(vectoring_in_vld),
    
    .vectoring_angle_i(vectoring_out_angle),
    .vectoring_out_vld_i(vectoring_out_vld),

    .ram_en_a_o(ram_en_a),
    .ram_we_a_o(ram_we_a),
    .ram_addr_a_o(ram_addr_a),
    .ram_din_a_o(ram_din_a),
    .ram_dout_a_i(ram_dout_a),

    .ram_en_b_o(ram_en_b),
    .ram_we_b_o(ram_we_b),
    .ram_addr_b_o(ram_addr_b),
    .ram_din_b_o(ram_din_b),
    .ram_dout_b_i(ram_dout_b),

    .rotation_in_dat_x_o(rotation_in_dat_x),
    .rotation_in_dat_y_o(rotation_in_dat_y),
    .rotation_in_dat_z_o(rotation_in_dat_z),
    .rotation_in_vld_o(rotation_in_vld),
    
    .rotation_fifo_out_dat_x_i(rotation_fifo_out_dat_x),
    .rotation_fifo_out_dat_y_i(rotation_fifo_out_dat_y),
    .rotation_fifo_out_dat_z_i(rotation_fifo_out_dat_z),
    .rotation_fifo_out_vld_i(rotation_fifo_out_vld)
  );

  calc_angle_pipeline calc_angle_pipeline_i (
    .clk(clk),
    .rst(rst),

    .x_i(vectoring_in_dat_x),
    .y_i(vectoring_in_dat_y),
    .vld_i(vectoring_in_vld),

    .angle_o(vectoring_out_angle),
    .vld_o(vectoring_out_vld)
  );

  dual_port_ram #(JACOBI_ADDR_WIDTH, JACOBI_MEM_SIZE, JACOBI_OUTPUT_WORD_WIDTH) bram0 (
    .clk_a(clk),
    .en_a(ram_en_a),
    .we_a(ram_we_a),
    .addr_a(ram_addr_a),
    .din_a(ram_din_a),
    .dout_a(ram_dout_a),

    .clk_b(clk),
    .en_b(ram_en_b),
    .we_b(ram_we_b),
    .addr_b(ram_addr_b),
    .din_b(ram_din_b),
    .dout_b(ram_dout_b)
  );

  cordic #(.MODE("rotation")) rotation_cordic (
    .clk(clk),
    .rst(rst),

    .x_i(rotation_in_dat_x),
    .y_i(rotation_in_dat_y),
    .z_i(rotation_in_dat_z),
    .vld_i(rotation_in_vld),

    .x_o(rotation_out_dat_x),
    .y_o(rotation_out_dat_y),
    .z_o(rotation_out_dat_z),
    .vld_o(rotation_out_vld)
  );

  /*
  FIFO (
    clk,
    rst,

    rotation_out_dat_x,
    rotation_out_dat_y,
    rotation_out_dat_z,
    
    rotation_fifo_out_dat_x,
    rotation_fifo_out_dat_y,
    rotation_fifo_out_dat_z
  )
  */

  //Temporary
  assign rotation_fifo_out_dat_x = rotation_out_dat_x;
  assign rotation_fifo_out_dat_y = rotation_out_dat_y;
  assign rotation_fifo_out_dat_z = rotation_out_dat_z;
  assign rotation_fifo_out_vld   = rotation_out_vld;

    
endmodule