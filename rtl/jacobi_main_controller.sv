import common::*;
module jacobi_main_controller (

  input                       clk,
  input                       rst, 

  // Input interface from microcontroller
  input  [JACOBI_INPUT_WORD_WIDTH-1:0]  in_dat_i,
  input                                 in_vld_i,
  output                                in_rdy_o,

  // Output interface to microcontroller
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] out_dat_o,
  output                                out_vld_o,
  input                                 out_rdy_i,

  // Output data to pipelined vectoring cordic (angle calc)
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_x_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_y_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_z_o,
  output                                vectoring_in_vld_o,

  // Input data from vectoring cordic
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_out_dat_x_i,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_out_dat_y_i,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_out_dat_z_i,
  output                                vectoring_out_vld_i,

  //Memory interface
  output                                ram_en_a_o,
  output                                ram_we_a_o,
  output [JACOBI_ADDR_WIDTH-1:0]        ram_addr_a_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_a_o,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_dout_a_i,

  output                                ram_en_b_o,
  output                                ram_we_b_o,
  output [JACOBI_ADDR_WIDTH-1:0]        ram_addr_b_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_b_o,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_dout_b_i,

  // Output data to rotation cordic
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_x_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_y_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_z_o,
  output                                rotation_in_vld_o,

  // Input data from rotation cordic output fifo
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_x_i,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_y_i,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_z_i,
  output                                rotation_fifo_out_vld_i
);
  
  
  /*********************
   * Signal declarations
  **********************/
  
  // Input rdy copy
  reg                            in_rdy;

  // Input to memory signals
  reg signed   [JACOBI_OUTPUT_WORD_WIDTH-1:0] in_dat_ram_din_a_r;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]        in_dat_ram_addr_a_r;
  reg                                         in_dat_ram_en_a_r;
  reg                                         in_dat_ram_we_a_r;

  reg signed   [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_a;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]        ram_addr_a;
  reg                                         ram_en_a;
  reg                                         ram_we_a;

  // Input data counter
  reg unsigned [JACOBI_LOG2_N_INPUT_DATA-1:0] in_dat_counter_r;

  /*******************
   * Type declarations
  *******************/
  typedef enum {INIT, IDLE, RECEIVE_DATA, FINISH_RECEIVING, CALC_ANGLES, CALC_VALUES, DRAW_ROUND, SEND_DATA} main_fsm_t; //
  main_fsm_t main_fsm_r;

  /***************
   * Instantations
  ***************/

  /*
  jacobi_addr_gen_lut a_port_addr_gen (
    .n(a_matrix_row),
    .k(a_matrix_col),
    .addr(ram_addr_a_o)
  );
  jacobi_addr_gen_lut b_port_addr_gen (
    .n(b_matrix_row),
    .k(b_matrix_col),
    .addr(ram_addr_b_o)
  );*/

  /******** Main FSM registered part ********/
  always_ff @(posedge clk) begin
    
    case(main_fsm_r)
      INIT: begin
        main_fsm_r <= IDLE;
      end
      IDLE: begin
        if (in_vld_i == 1) begin
          main_fsm_r <= RECEIVE_DATA;
        end
      end

      RECEIVE_DATA: begin
        if (in_vld_i == 1 && in_rdy == 1 && in_dat_counter_r == JACOBI_N_INPUT_DATA-1) begin
          main_fsm_r <= FINISH_RECEIVING;
        end
      end

      FINISH_RECEIVING: begin
        main_fsm_r <= CALC_ANGLES;
      end
  
      
      CALC_ANGLES:
        main_fsm_r <= CALC_ANGLES;
      
      CALC_VALUES:
        main_fsm_r <= CALC_VALUES;

      DRAW_ROUND:
        main_fsm_r <= DRAW_ROUND;

      SEND_DATA:
        main_fsm_r <= SEND_DATA;
    endcase

    if (rst == 1) begin
      main_fsm_r <= INIT;
    end

  end

  always_comb begin : main_fsm_comb

    if (main_fsm_r == IDLE || main_fsm_r == RECEIVE_DATA) begin
      in_rdy <= 1;
    end else begin
      in_rdy <= 0;
    end

    if (main_fsm_r == IDLE || main_fsm_r == RECEIVE_DATA || main_fsm_r == FINISH_RECEIVING) begin
      ram_en_a   <= in_dat_ram_en_a_r;
      ram_we_a   <= in_dat_ram_we_a_r;
      ram_addr_a <= in_dat_ram_addr_a_r;
      ram_din_a  <= in_dat_ram_din_a_r;
    end else begin
      ram_en_a   <= 0;
      ram_we_a   <= 0;
      ram_addr_a <= 0;
      ram_din_a  <= 0;
    end


  end


  /******** Receive data from uc ********/
  always_ff @(posedge clk) begin : receive_from_uc_p
    if (in_vld_i == 1 && in_rdy == 1) begin
      in_dat_ram_din_a_r <= in_dat_i;
      in_dat_ram_addr_a_r <= in_dat_counter_r;
      in_dat_ram_en_a_r <= 1;
      in_dat_ram_we_a_r <= 1;
    end else begin
      in_dat_ram_en_a_r <= 0;
      in_dat_ram_we_a_r <= 0;
    end

    if (rst == 1) begin
      in_dat_ram_en_a_r <= 0;
      in_dat_ram_we_a_r <= 0;
    end
  end

  /******** Input data counter ********/
  always_ff @(posedge clk) begin : in_dat_counter_p
    if (in_vld_i == 1 && in_rdy == 1) begin
      if (in_dat_counter_r == JACOBI_N_INPUT_DATA - 1) begin
        in_dat_counter_r <= 0;
      end else begin
        in_dat_counter_r <= in_dat_counter_r + 1;
      end
    end

    if (rst == 1) begin
      in_dat_counter_r <= 0;
    end
  end


  assign in_rdy_o = in_rdy;

  assign ram_din_a_o = ram_din_a;
  assign ram_addr_a_o = ram_addr_a;
  assign ram_en_a_o = ram_en_a;
  assign ram_we_a_o = ram_we_a;

endmodule



