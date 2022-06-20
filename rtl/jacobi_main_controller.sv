import common::*;
module jacobi_main_controller (

  input                       clk,
  input                       rst, 

  // Input interface from microcontroller
  input signed [JACOBI_INPUT_WORD_WIDTH-1:0]  in_dat_i,
  input                                 in_vld_i,
  output                                in_rdy_o,

  // Output interface to microcontroller
  output signed [31:0]                  out_dat_o,
  output                                out_vld_o,
  output reg                            out_last_o,
  input                                 out_rdy_i,

  // Output data to pipelined vectoring cordic (angle calc)
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_x_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_y_o,
  output                                vectoring_in_vld_o,

  // Input data from vectoring cordic
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_angle_i,
  input                                 vectoring_out_vld_i,

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
  input                                 rotation_fifo_out_vld_i
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

  // Indices and angles array 
  reg unsigned [JACOBI_LOG2_N-1:0] indices_shift_register_r [JACOBI_N_PAIRS-1:0][JACOBI_PAIR-1:0];
  reg signed   [JACOBI_OUTPUT_WORD_WIDTH-1:0] angles_register_r [JACOBI_N_PAIRS-1:0];

  // Angle calculation
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_a_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_a_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   angle_ram_addr_a;
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_b_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       angle_b_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   angle_ram_addr_b;

  reg unsigned [JACOBI_LOG2_N_PAIRS-1:0] calc_angles_push_pair_counter_r;
  reg unsigned [JACOBI_LOG2_PAIR-1:0]    calc_angles_push_phase_counter_r;
  reg unsigned [JACOBI_LOG2_N_PAIRS-1:0] calc_angle_phase_counter_one_cycle_delayed_r;
  reg unsigned [JACOBI_LOG2_N_PAIRS-1:0] calc_angle_phase_counter_two_cycles_delayed_r;

  reg unsigned [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_x_r;
  reg unsigned [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_y_r;
  reg unsigned                                vectoring_in_vld_r;
  
  reg unsigned [JACOBI_LOG2_N_PAIRS-1:0] calc_angles_store_counter_r;
  
  reg unsigned [31:0] send_data_counter_r;

  reg out_vld_s;



  /*******************
   * Type declarations
  *******************/
  typedef enum {INIT, IDLE, RECEIVE_DATA, CALC_ANGLES_PUSH, CALC_ANGLES_STORE, CALC_VALUES, DRAW_ROUND, SEND_DATA} main_fsm_t; //
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
            main_fsm_r <= CALC_ANGLES_PUSH;
          end
        end else begin
          ram_en_a_r <= 0;
          ram_we_a_r <= 0;
        end
      end
  
      
      CALC_ANGLES_PUSH: begin 
        // Pairs are shifted in circular pattern to the right in order to push subsequent pairs to pipeline
        if (calc_angles_push_phase_counter_r == JACOBI_PAIR-1) begin
          indices_shift_register_r[0] <= indices_shift_register_r[1];
          indices_shift_register_r[1] <= indices_shift_register_r[2];
          indices_shift_register_r[2] <= indices_shift_register_r[3];
          indices_shift_register_r[3] <= indices_shift_register_r[0];
        end
        
        ram_en_a_r   <= 1;
        ram_we_a_r   <= 0;
        ram_addr_a_r <= angle_ram_addr_a;
        ram_en_b_r   <= 1;
        ram_we_b_r   <= 0;
        ram_addr_b_r <= angle_ram_addr_b;

        if (calc_angles_push_pair_counter_r == JACOBI_N_PAIRS-1 && calc_angles_push_phase_counter_r == JACOBI_PAIR-1) begin
          main_fsm_r <= CALC_ANGLES_STORE;
        end
      end


      CALC_ANGLES_STORE: begin //read from pipeline and store in registers
        ram_en_a_r <= 0;
        ram_en_b_r <= 0;
        if (vectoring_out_vld_i == 1) begin
          angles_register_r[3] <= vectoring_angle_i;
          angles_register_r[2] <= angles_register_r[3];
          angles_register_r[1] <= angles_register_r[2];
          angles_register_r[0] <= angles_register_r[1];
          if (calc_angles_store_counter_r == JACOBI_N_PAIRS-1) begin
            main_fsm_r <= CALC_VALUES;
          end
        end
      end

      CALC_VALUES:
        main_fsm_r <= DRAW_ROUND;

      DRAW_ROUND: begin
        indices_shift_register_r[1][0] <= indices_shift_register_r[2][0];
        indices_shift_register_r[2][0] <= indices_shift_register_r[3][0];
        indices_shift_register_r[3][0] <= indices_shift_register_r[3][1];
        indices_shift_register_r[3][1] <= indices_shift_register_r[2][1];
        indices_shift_register_r[2][1] <= indices_shift_register_r[1][1];
        indices_shift_register_r[1][1] <= indices_shift_register_r[0][1];
        indices_shift_register_r[0][1] <= indices_shift_register_r[1][0];
        main_fsm_r <= SEND_DATA;
      end
        

      SEND_DATA: begin
        //ram_en_b_r <= 1;
        //ram_addr_b_r <= send_data_counter_r;
        if (send_data_counter_r == 99 && out_vld_s == 1) begin
          main_fsm_r <= INIT;
        end
      end
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

  /******** Combinational process for main FSM ********/
  always_comb begin : main_fsm_comb
    
    // Input ready
    if (main_fsm_r == IDLE || main_fsm_r == RECEIVE_DATA) begin
      in_rdy <= 1;
    end else begin
      in_rdy <= 0;
    end
    
    //CALC_ANGLES- address computation
    if (calc_angles_push_phase_counter_r == 0) begin //Wjj
      angle_a_matrix_col <= indices_shift_register_r[0][1];
      angle_a_matrix_row <= indices_shift_register_r[0][1];
    end else begin //Wii
      angle_a_matrix_col <= indices_shift_register_r[0][0];
      angle_a_matrix_row <= indices_shift_register_r[0][0];
    end
    angle_b_matrix_row <= indices_shift_register_r[0][0];
    angle_b_matrix_col <= indices_shift_register_r[0][1];

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
  always_ff @(posedge clk) begin : calc_angle_push_phase_counter_p
    if (main_fsm_r == CALC_ANGLES_PUSH) begin
      if (calc_angles_push_phase_counter_r == JACOBI_PAIR-1) begin
        calc_angles_push_phase_counter_r <= 0;
      end else begin
        calc_angles_push_phase_counter_r <= calc_angles_push_phase_counter_r + 1;
      end
    end else begin
      calc_angles_push_phase_counter_r <= 0;
    end
    calc_angle_phase_counter_one_cycle_delayed_r <= calc_angles_push_phase_counter_r;
    calc_angle_phase_counter_two_cycles_delayed_r <= calc_angle_phase_counter_one_cycle_delayed_r;
    if (rst == 1) begin
      calc_angles_push_phase_counter_r <= 0;
      calc_angle_phase_counter_one_cycle_delayed_r <= 0;
      calc_angle_phase_counter_two_cycles_delayed_r <= 0;
    end
    
  end

  /******** Calc Angles Push Pair counter ********/
  always_ff @(posedge clk) begin : calc_angle_push_pair_counter_p
    if (main_fsm_r == CALC_ANGLES_PUSH) begin
      if (calc_angles_push_phase_counter_r == JACOBI_PAIR-1) begin
        if (calc_angles_push_pair_counter_r == JACOBI_N_PAIRS-1) begin
          calc_angles_push_pair_counter_r <= 0;
        end else begin
          calc_angles_push_pair_counter_r <= calc_angles_push_pair_counter_r + 1;
        end
      end
    end else begin
      calc_angles_push_pair_counter_r <= 0;
    end

    if (rst == 1) begin
      calc_angles_push_pair_counter_r <= 0;
    end 
  end
  
  always_ff @(posedge clk) begin : calc_angle_push_data_to_cordic_p
    // Calculate 2Wij and Wjj-Wii and push data to cordic
    if (calc_angle_phase_counter_two_cycles_delayed_r == 0 && calc_angle_phase_counter_one_cycle_delayed_r == 1) begin
      vectoring_in_dat_x_r <= ram_dout_a_i;
      vectoring_in_dat_y_r <= ram_dout_b_i;
      vectoring_in_vld_r   <= 0;
    end else if (calc_angle_phase_counter_two_cycles_delayed_r == 1) begin
      vectoring_in_dat_x_r <= vectoring_in_dat_x_r - ram_dout_a_i;
      vectoring_in_dat_y_r <= vectoring_in_dat_y_r + ram_dout_b_i;
      vectoring_in_vld_r   <= 1;
    end else begin
      vectoring_in_vld_r   <= 0;
    end
  end

  /******** Calc Angles Store Counter ********/
  always_ff @(posedge clk) begin : calc_angles_store_counter_p 
    if (vectoring_out_vld_i == 1) begin
      if (calc_angles_store_counter_r == JACOBI_N_PAIRS-1) begin
        calc_angles_store_counter_r <= 0;
      end else begin
        calc_angles_store_counter_r <= calc_angles_store_counter_r + 1;
      end
    end

    if (rst == 1) begin
      calc_angles_store_counter_r <= 0;
    end
  end

  always_ff @(posedge clk) begin : send_data_counter_p
    if (out_vld_s == 1 && out_rdy_i == 1) begin
      if (send_data_counter_r == 99) begin
        send_data_counter_r <= 0;
      end else begin
        send_data_counter_r <= send_data_counter_r + 1;
      end
    end

    if (rst == 1) begin
      send_data_counter_r <= 0;
    end

  end

/*  always_ff @(posedge clk) begin : display_valid_p
    if ((send_data_counter_r > 0 || send_data_counter_two_cycles_delayed_r > 0) && send_data_counter_two_cycles_delayed_r < 99) begin
      vld_out_r <= 1;
    end else begin
      vld_out_r <= 0;
    end
  end*/
  always_comb begin
    if (main_fsm_r == SEND_DATA) begin
      out_vld_s = 1;
    end else begin
      out_vld_s = 0;
    end
    
    if (send_data_counter_r == 99) begin
      out_last_o <= 1;
    end else begin
      out_last_o <= 0;
    end
    
  end

  assign in_rdy_o = in_rdy;

  assign ram_din_a_o = ram_din_a_r;
  assign ram_addr_a_o = ram_addr_a_r;
  assign ram_en_a_o = ram_en_a_r;
  assign ram_we_a_o = ram_we_a_r;

  assign ram_din_b_o = ram_din_b_r;
  assign ram_addr_b_o = ram_addr_b_r;
  assign ram_en_b_o = ram_en_b_r;
  assign ram_we_b_o = ram_we_b_r;

  assign vectoring_in_dat_x_o = vectoring_in_dat_x_r;
  assign vectoring_in_dat_y_o = vectoring_in_dat_y_r;
  assign vectoring_in_vld_o = vectoring_in_vld_r;

  assign out_vld_o = out_vld_s;
  assign out_dat_o = send_data_counter_r;

endmodule



