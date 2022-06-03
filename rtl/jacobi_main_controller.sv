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
  reg signed   [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_a_r;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]        ram_addr_a_r;
  reg                                         ram_en_a_r;
  reg                                         ram_we_a_r;

  reg signed   [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_b_r;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]        ram_addr_b_r;
  reg                                         ram_en_b_r;
  reg                                         ram_we_b_r;

  // Init counters
  reg unsigned [JACOBI_LOG2_N-1:0] init_row_counter_r;
  reg unsigned [JACOBI_LOG2_N-1:0] init_col_counter_r;

  // Input data counter
  reg unsigned [JACOBI_LOG2_N_INPUT_DATA-1:0] in_dat_counter_r;

  // Idx array
  reg unsigned [JACOBI_LOG2_N-1:0] indices_shift_register_r [JACOBI_N_PAIRS-1:0][JACOBI_PAIR-1:0];

  // Angle calculation
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_a_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_a_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   angle_ram_addr_a;
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_b_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_b_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   angle_ram_addr_b;
  reg unsigned [JACOBI_LOG2_N_PAIRS-1:0] calc_angle_pair_counter_r
  reg unsigned [JACOBI_LOG2_PAIR-1:0]    calc_angle_phase_counter_r


  /*******************
   * Type declarations
  *******************/
  typedef enum {INIT, IDLE, RECEIVE_DATA, FINISH_RECEIVING, CALC_ANGLES, CALC_VALUES, DRAW_ROUND, SEND_DATA} main_fsm_t; //
  main_fsm_t main_fsm_r;

  /***************
   * Instantations
  ***************/

  // Address generators for vectoring mode cordic
  jacobi_addr_gen_lut a_port_angle_calc_addr_gen (
    .n(angle_a_matrix_row),
    .k(angle_a_matrix_col),
    .addr(angle_ram_addr_a)
  );
  jacobi_addr_gen_lut b_port_angle_calc_addr_gen (
    .n(angle_b_matrix_row),
    .k(angle_b_matrix_col),
    .addr(angle_ram_addr_b)
  );

  /******** Main FSM registered part ********/
  always_ff @(posedge clk) begin
    
    case(main_fsm_r)
      INIT: begin //Initialize V matrix with eye matrix
        // Place 1 on main diagonal
        if (init_row_counter_r == init_col_counter_r) begin
          ram_din_a_r <= 1;
        end else begin
          ram_din_a_r <= 0;
        end
        // Remark: Address is equal to 64 + 8*row + column
        ram_addr_a_r <= (init_row_counter_r <<< 3) + init_col_counter_r + JACOBI_V_OFFSET;
        ram_en_a_r <= 1;
        ram_we_a_r <= 1;

        if (init_row_counter_r == JACOBI_N-1 && init_col_counter_r == JACOBI_N-1) begin
          main_fsm_r <= IDLE;
        end
      end
      
      IDLE: begin //Wait for master to transmit data
        if (in_vld_i == 1 && in_rdy == 1) begin
          ram_din_a_r <= in_dat_i;
          ram_addr_a_r <= in_dat_counter_r;
          ram_en_a_r <= 1;
          ram_we_a_r <= 1;
          main_fsm_r <= RECEIVE_DATA;
        end
      end

      RECEIVE_DATA: begin //Receive data and store in ram
        if (in_vld_i == 1 && in_rdy == 1) begin
          ram_din_a_r <= in_dat_i;
          ram_addr_a_r <= in_dat_counter_r;
          ram_en_a_r <= 1;
          ram_we_a_r <= 1;
          if (in_dat_counter_r == JACOBI_N_INPUT_DATA-1) begin
            main_fsm_r <= FINISH_RECEIVING;
          end
        end else begin
          ram_en_a_r <= 0;
          ram_we_a_r <= 0;
        end
      end

      FINISH_RECEIVING: begin
        ram_en_a_r <= 0;
        ram_we_a_r <= 0;
        main_fsm_r <= CALC_ANGLES;
      end
  
      
      CALC_ANGLES_PUSH: // Pairs are shifted in circular pattern to the right in order to push subsequent pairs to pipeline

        if (calc_angle_phase_counter_r == JACOBI_PAIR-1) begin
          indices_shift_register_r[0] <= indices_shift_register_r[1];
          indices_shift_register_r[1] <= indices_shift_register_r[2];
          indices_shift_register_r[2] <= indices_shift_register_r[3];
          indices_shift_register_r[3] <= indices_shift_register_r[0];

          if (calc_angle_pair_counter_r == JACOBI_N_PAIRS-1) begin
            main_fsm_r <= CALC_ANGLES_WAIT;
          end
        end
        ram_en_a_r   <= 1;
        ram_we_a_r   <= 0;
        ram_addr_a_r <= angle_ram_addr_a;
        ram_en_b_r   <= 1;
        ram_en_b_r   <= 0;
        ram_addr_b_r <= angle_ram_addr_b;


      CALC_ANGLES_WAIT:
        main_fsm_r <= CALC_ANGLES_WAIT;

      CALC_VALUES:
        main_fsm_r <= CALC_VALUES;

      DRAW_ROUND:
        main_fsm_r <= DRAW_ROUND;

      SEND_DATA:
        main_fsm_r <= SEND_DATA;
    endcase

    if (rst == 1) begin
      main_fsm_r <= INIT;
      ram_en_a_r <= 0;
      ram_we_a_r <= 0;
      ram_en_b_r <= 0;
      ram_we_b_r <= 0;
      indices_shift_register_r[0][0] <= 0;
      indices_shift_register_r[0][1] <= 7;
      indices_shift_register_r[1][0] <= 1;
      indices_shift_register_r[1][1] <= 6;
      indices_shift_register_r[2][0] <= 2;
      indices_shift_register_r[2][1] <= 5;
      indices_shift_register_r[3][0] <= 3;
      indices_shift_register_r[3][1] <= 4;
      
    end

  end

  always_comb begin : main_fsm_comb

    if (main_fsm_r == IDLE || main_fsm_r == RECEIVE_DATA) begin
      in_rdy <= 1;
    end else begin
      in_rdy <= 0;
    end

  end

  /******** Init column counter ********/
  always_ff @(posedge clk) begin : init_col_counter_p
    if (main_fsm_r == INIT) begin
      if (init_col_counter_r == JACOBI_N-1) begin
        init_col_counter_r <= 0;
      end else begin
        init_col_counter_r <= init_col_counter_r + 1;
      end
    end

    if (rst == 1) begin
      init_col_counter_r <= 0;
    end
  end

  /******** Init row counter ********/
  always_ff @(posedge clk) begin : init_row_counter_p
    if (main_fsm_r == INIT) begin
      if (init_col_counter_r == JACOBI_N-1) begin
        if (init_row_counter_r == JACOBI_N-1) begin
          init_row_counter_r <= 0;
        end else begin
          init_row_counter_r <= init_row_counter_r + 1;
        end
      end
    end

    if (rst == 1) begin
      init_row_counter_r <= 0;
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

  /******** Calc Angles Phase counter ********/ 
  // Phase 0: Fetch diagonal elements (ii,jj), Phase 1: Fetch no-diagonal element (i,j)
  always_ff @(posedge clk) begin : calc_angle_phase_counter_p
    if (main_fsm_r == CALC_ANGLES) begin
      if (calc_angle_phase_counter_r == JACOBI_PAIR-1) begin
        calc_angle_phase_counter_r <= 0;
      end else begin
        calc_angle_phase_counter_r <= calc_angle_phase_counter_r + 1;
      end
    end

    if (rst == 1) begin
      calc_angle_phase_counter_r <= 0;
    end
  end

  /******** Calc Angles Pair counter ********/
  always_ff @(posedge clk) begin : calc_angle_pair_counter_p
    if (main_fsm_r == CALC_ANGLES) begin
      if (calc_angle_phase_counter_r == JACOBI_PAIR-1) begin
        if (calc_angle_pair_counter_r == JACOBI_N_PAIRS-1) begin
          calc_angle_pair_counter_r <= 0;
        end else begin
          calc_angle_pair_counter_r <= calc_angle_pair_counter_r + 1;
        end
      end
    end

    if (rst == 1) begin
      calc_angle_pair_counter_r <= 0;
    end 
  end

  // Address generator for angle calc
  always_comb begin : calc_angles_addr_gen_p
    angle_a_matrix_row <= indices_shift_register_r[0][0];
    if (calc_angle_phase_counter_r == 0) begin      
      angle_a_matrix_col <= indices_shift_register_r[0][0];
    end else begin
      angle_a_matrix_col <= indices_shift_register_r[0][1];
    end
    angle_b_matrix_row <= indices_shift_register_r[0][1];
    angle_b_matrix_row <= indices_shift_register_r[0][1];
  end



  assign in_rdy_o = in_rdy;

  assign ram_din_a_o = ram_din_a_r;
  assign ram_addr_a_o = ram_addr_a_r;
  assign ram_en_a_o = ram_en_a_r;
  assign ram_we_a_o = ram_we_a_r;

endmodule



