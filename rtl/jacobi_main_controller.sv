import common::*;
module jacobi_main_controller (

  input                       clk,
  input                       rst, 

  // Input interface from microcontroller
  input signed [JACOBI_INPUT_WORD_WIDTH-1:0]  in_dat_i,
  input                                 in_vld_i,
  output                                in_rdy_o,

  // Output interface to microcontroller
  output signed [AXI4_FIFO_WORD_WIDTH-1:0]     out_dat_o,
  output                                       out_vld_o,
  output                                       out_last_o,
  input                                        out_rdy_i,


  // Output data to pipelined vectoring cordic (angle calc)
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_x_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_in_dat_y_o,
  output                                vectoring_in_vld_o,

  // Input data from vectoring cordic
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] vectoring_angle_i,
  input                                 vectoring_out_vld_i,

  //Memory interface
  output                                       ram_en_a_o,
  output                                       ram_we_a_o,
  output        [JACOBI_ADDR_WIDTH-1:0]        ram_addr_a_o,
  output signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_a_o,
  input  signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_dout_a_i,

  output                                       ram_en_b_o,
  output                                       ram_we_b_o,
  output        [JACOBI_ADDR_WIDTH-1:0]        ram_addr_b_o,
  output signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_din_b_o,
  input  signed [JACOBI_OUTPUT_WORD_WIDTH-1:0] ram_dout_b_i,

  // Output data to rotation cordic
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_x_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_y_o,
  output [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_z_o,
  output                                rotation_in_vld_o,

  // Input data from rotation cordic output fifo
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_x_i,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_y_i,
  input  [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_z_i,
  input                                 rotation_fifo_out_vld_i,
  output                                rotation_fifo_out_rdy_o
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

  reg unsigned [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_x_r;
  reg unsigned [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_y_r;
  reg unsigned [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_in_dat_z_r;
  reg unsigned                                rotation_in_vld_r;

  reg unsigned                                rotation_fifo_out_rdy_r;
  
  reg unsigned [JACOBI_LOG2_N_PAIRS-1:0] calc_angles_store_counter_r;

  // Value calculation
  reg unsigned [JACOBI_LOG2_N-1:0]       value_a_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       value_a_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   value_ram_addr_a;
  reg unsigned [JACOBI_LOG2_N-1:0]       value_b_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       value_b_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   value_ram_addr_b;

  // Writing to memory
  reg unsigned [JACOBI_LOG2_N-1:0]       value_write_a_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       value_write_a_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   value_write_ram_addr_a;
  reg unsigned [JACOBI_LOG2_N-1:0]       value_write_b_matrix_col;
  reg unsigned [JACOBI_LOG2_N-1:0]       value_write_b_matrix_row;
  reg unsigned [JACOBI_ADDR_WIDTH-1:0]   value_write_ram_addr_b;

  reg unsigned [4:0]                     calc_values_in_one_pair_counter_r;
  reg unsigned [JACOBI_LOG2_N-1:0]       calc_values_column_ind_r;
  reg unsigned                           calc_values_push_values_to_cordic_r;
  reg unsigned                           calc_values_push_values_to_cordic_one_cycle_delayed_r;
  reg unsigned [2:0]                     calc_values_get_matrix_data_row_counter_r;
  
  // pushing to cordic
  reg unsigned [4:0]                     calc_values_cordic_receive_counter_r;
  reg unsigned [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_x_temp_r;
  reg unsigned [JACOBI_OUTPUT_WORD_WIDTH-1:0] rotation_fifo_out_dat_y_temp_r;
  reg unsigned                           calc_values_valid_r;
  reg unsigned [4:0]                     calc_values_save_data_from_cordic_counter_r;

  reg unsigned [JACOBI_LOG2_N-1:0]       calc_values_write_column_ind_r;
  reg unsigned                           calc_values_saving_finished_r;

  reg unsigned [JACOBI_LOG2_N_PAIRS-1:0] change_pair_pair_counter_r;
  reg unsigned [5:0]                     draw_round_counter_r;

  reg unsigned [2:0]                     calc_values_write_matrix_data_row_counter_r;

  reg unsigned [JACOBI_LOG2_N-1:0] current_indices_in_order_r [JACOBI_PAIR-1:0];


  // Send data
  reg unsigned [JACOBI_ADDR_WIDTH-1:0] send_data_counter_r;
  reg                                  sender_vld_s;
  reg                                  sender_vld_r;
  reg                                  out_vld_r;
  reg                                  last_s;
  reg                                  last_r;
  reg                                  out_last_r;



  /*******************
   * Type declarations
  *******************/
  typedef enum {INIT, IDLE, RECEIVE_DATA, CALC_ANGLES_PUSH, CALC_ANGLES_STORE, CALC_VALUES, CHANGE_PAIR, DRAW_ROUND, SEND_DATA, FINISH_SENDING} main_fsm_t; //
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

  // Address generators for rotation mode cordic
  jacobi_addr_gen_lut a_port_value_calc_addr_gen (
    .n(value_a_matrix_row),
    .k(value_a_matrix_col),
    .addr(value_ram_addr_a)
  );
  jacobi_addr_gen_lut b_port_value_calc_addr_gen (
    .n(value_b_matrix_row),
    .k(value_b_matrix_col),
    .addr(value_ram_addr_b)
  );

  // Address generators for writing to memory
  jacobi_addr_gen_lut a_port_value_write_calc_addr_gen (
    .n(value_write_a_matrix_row),
    .k(value_write_a_matrix_col),
    .addr(value_write_ram_addr_a)
  );
  jacobi_addr_gen_lut b_port_value_write_calc_addr_gen (
    .n(value_write_b_matrix_row),
    .k(value_write_b_matrix_col),
    .addr(value_write_ram_addr_b)
  );

  /******** Main FSM registered part ********/
  always_ff @(posedge clk) begin
    
    case(main_fsm_r)
      INIT: begin //Initialize V matrix with eye matrix
        // Place 1 on main diagonal
        if (init_row_counter_r == init_col_counter_r) begin
          ram_din_a_r <= 1 <<< 15;
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


      CALC_VALUES: begin
        
        //Receive data from memory and push to cordic
        if (calc_values_in_one_pair_counter_r < JACOBI_N) begin
          ram_en_a_r   <= 1;
          ram_we_a_r   <= 0;
          ram_addr_a_r <= value_ram_addr_a;
          ram_en_b_r   <= 1;
          ram_we_b_r   <= 0;
          ram_addr_b_r <= value_ram_addr_b;
          calc_values_push_values_to_cordic_r <= 1;
        end else if (calc_values_in_one_pair_counter_r < JACOBI_N <<< 1) begin
          ram_en_a_r   <= 1;
          ram_we_a_r   <= 0;
          ram_addr_a_r <= (calc_values_get_matrix_data_row_counter_r <<< 3) + current_indices_in_order_r[0] + JACOBI_V_OFFSET;
          ram_en_b_r   <= 1;
          ram_we_b_r   <= 0;
          ram_addr_b_r <= (calc_values_get_matrix_data_row_counter_r <<< 3) + current_indices_in_order_r[1] + JACOBI_V_OFFSET;
          calc_values_push_values_to_cordic_r <= 1;
          calc_values_get_matrix_data_row_counter_r <= calc_values_get_matrix_data_row_counter_r + 1;
        end else begin
          calc_values_push_values_to_cordic_r <= 0;
        end

        //Write matrix
        if (calc_values_valid_r) begin
          if (calc_values_save_data_from_cordic_counter_r < JACOBI_N - 2 || calc_values_save_data_from_cordic_counter_r == (JACOBI_N <<< 1) - 2) begin
            ram_en_a_r   <= 1;
            ram_we_a_r   <= 1;
            ram_addr_a_r <= value_write_ram_addr_a;
            ram_din_a_r  <= rotation_fifo_out_dat_x_temp_r;

            ram_en_b_r   <= 1;
            ram_we_b_r   <= 1;
            ram_addr_b_r <= value_write_ram_addr_b;
            ram_din_b_r  <= rotation_fifo_out_dat_y_temp_r;
          end else if (calc_values_save_data_from_cordic_counter_r < (JACOBI_N <<< 1) - 2) begin
            ram_en_a_r   <= 1;
            ram_we_a_r   <= 1;
            ram_addr_a_r <= (calc_values_write_matrix_data_row_counter_r <<< 3) + current_indices_in_order_r[0] + JACOBI_V_OFFSET;
            ram_din_a_r  <= rotation_fifo_out_dat_x_temp_r;

            ram_en_b_r   <= 1;
            ram_we_b_r   <= 1;
            ram_addr_b_r <= (calc_values_write_matrix_data_row_counter_r <<< 3) + current_indices_in_order_r[1] + JACOBI_V_OFFSET;
            ram_din_b_r  <= rotation_fifo_out_dat_y_temp_r;

            calc_values_write_matrix_data_row_counter_r <= calc_values_write_matrix_data_row_counter_r + 1;
          end
        end else if (calc_values_save_data_from_cordic_counter_r == (JACOBI_N <<< 1) - 1) begin
          ram_en_a_r   <= 1;
          ram_we_a_r   <= 1;
          ram_addr_a_r <= value_write_ram_addr_a;
          ram_din_a_r  <= 0;

          ram_we_b_r <= 0;
          
          calc_values_saving_finished_r <= 1;
          calc_values_write_matrix_data_row_counter_r <= 0;

        end

        if (calc_values_saving_finished_r) begin
          main_fsm_r <= CHANGE_PAIR;
          ram_we_a_r <= 0; 
        end
        
      end

      CHANGE_PAIR: begin
        calc_values_saving_finished_r <= 0;
        angles_register_r[0] <= angles_register_r[1];
        angles_register_r[1] <= angles_register_r[2];
        angles_register_r[2] <= angles_register_r[3];
        angles_register_r[3] <= angles_register_r[0];
        
        indices_shift_register_r[0] <= indices_shift_register_r[1];
        indices_shift_register_r[1] <= indices_shift_register_r[2];
        indices_shift_register_r[2] <= indices_shift_register_r[3];
        indices_shift_register_r[3] <= indices_shift_register_r[0];

        if (change_pair_pair_counter_r < (JACOBI_N >>> 1) - 1) begin
          change_pair_pair_counter_r <= change_pair_pair_counter_r + 1;
          main_fsm_r <= CALC_VALUES;
        end else begin
          change_pair_pair_counter_r <= 0;
          main_fsm_r <= DRAW_ROUND;
        end
      end

      DRAW_ROUND: begin
        indices_shift_register_r[1][0] <= indices_shift_register_r[2][0];
        indices_shift_register_r[2][0] <= indices_shift_register_r[3][0];
        indices_shift_register_r[3][0] <= indices_shift_register_r[3][1];
        indices_shift_register_r[3][1] <= indices_shift_register_r[2][1];
        indices_shift_register_r[2][1] <= indices_shift_register_r[1][1];
        indices_shift_register_r[1][1] <= indices_shift_register_r[0][1];
        indices_shift_register_r[0][1] <= indices_shift_register_r[1][0];

        if (draw_round_counter_r < JACOBI_N_ROUNDS - 1) begin
          draw_round_counter_r <= draw_round_counter_r + 1;
          main_fsm_r <= CALC_ANGLES_PUSH;
        end else begin
          draw_round_counter_r <= 0;
          main_fsm_r <= SEND_DATA;
        end
      end
        

      SEND_DATA: begin
        ram_en_a_r <= 1;
        ram_addr_a_r <= send_data_counter_r;
        ram_en_b_r <= 0;

        if (send_data_counter_r == JACOBI_MEM_SIZE-1) begin
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
      draw_round_counter_r <= 0;
      change_pair_pair_counter_r <= 0;
      calc_values_push_values_to_cordic_r <= 0;
      calc_values_get_matrix_data_row_counter_r <= 0;
      calc_values_saving_finished_r <= 0;
      calc_values_write_matrix_data_row_counter_r <= 0;
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

    // Sort indices
    if (indices_shift_register_r[0][0] < indices_shift_register_r[0][1]) begin
      current_indices_in_order_r[0] <= indices_shift_register_r[0][0];
      current_indices_in_order_r[1] <= indices_shift_register_r[0][1];
    end else begin
      current_indices_in_order_r[0] <= indices_shift_register_r[0][1];
      current_indices_in_order_r[1] <= indices_shift_register_r[0][0];
    end
    
    //CALC_ANGLES- address computation
    if (calc_angles_push_phase_counter_r == 0) begin //Wjj
      angle_a_matrix_col <= current_indices_in_order_r[1];
      angle_a_matrix_row <= current_indices_in_order_r[1];
    end else begin //Wii
      angle_a_matrix_col <= current_indices_in_order_r[0];
      angle_a_matrix_row <= current_indices_in_order_r[0];
    end
    angle_b_matrix_row <= current_indices_in_order_r[0];
    angle_b_matrix_col <= current_indices_in_order_r[1];

    //CALC_VALUES - getting values address computation
    if (calc_values_in_one_pair_counter_r == 0) begin
      value_a_matrix_row <= current_indices_in_order_r[0];
      value_a_matrix_col <= current_indices_in_order_r[0];
      value_b_matrix_row <= current_indices_in_order_r[0];
      value_b_matrix_col <= current_indices_in_order_r[1];

    end else if (calc_values_in_one_pair_counter_r == 1) begin
      value_a_matrix_row <= current_indices_in_order_r[0];
      value_a_matrix_col <= current_indices_in_order_r[1];
      value_b_matrix_row <= current_indices_in_order_r[1];
      value_b_matrix_col <= current_indices_in_order_r[1];
    end else begin
      value_a_matrix_row <= current_indices_in_order_r[0];
      value_a_matrix_col <= calc_values_column_ind_r;
      value_b_matrix_row <= current_indices_in_order_r[1];
      value_b_matrix_col <= calc_values_column_ind_r;
    end

    //CALC_VALUES - writing values address computation
    if (calc_values_save_data_from_cordic_counter_r < (JACOBI_N <<< 1) - 2) begin
      value_write_a_matrix_row <= current_indices_in_order_r[0];
      value_write_a_matrix_col <= calc_values_write_column_ind_r;
      value_write_b_matrix_row <= current_indices_in_order_r[1];
      value_write_b_matrix_col <= calc_values_write_column_ind_r;
    end else if (calc_values_save_data_from_cordic_counter_r == (JACOBI_N <<< 1) - 2) begin
      value_write_a_matrix_row <= current_indices_in_order_r[0];
      value_write_a_matrix_col <= current_indices_in_order_r[0];
      value_write_b_matrix_row <= current_indices_in_order_r[1];
      value_write_b_matrix_col <= current_indices_in_order_r[1];
    end else begin
      value_write_a_matrix_row <= current_indices_in_order_r[0];
      value_write_a_matrix_col <= current_indices_in_order_r[1];
    end


    if (main_fsm_r == SEND_DATA) begin
      sender_vld_s = 1;

      if (send_data_counter_r == JACOBI_MEM_SIZE-1) begin
        last_s <= 1;
      end else begin
        last_s <= 0;
      end
      
    end else begin
      sender_vld_s = 0;
      last_s <= 0;
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



  /********* Calc values in one pair counter **/
  always_ff @(posedge clk) begin : calc_values_in_one_pair_cnt_p
    if (main_fsm_r == CALC_VALUES) begin
      if (calc_values_in_one_pair_counter_r < JACOBI_N <<< 1) begin
        calc_values_in_one_pair_counter_r <= calc_values_in_one_pair_counter_r + 1;
      end
    end else begin
      calc_values_in_one_pair_counter_r <= 0;
    end


    if (rst == 1) begin
      calc_values_in_one_pair_counter_r <= 0;
    end
  end

  /******** Calc values calc column indexes **/
  always_ff @(posedge clk) begin : calc_values_column_ind_p
    if (main_fsm_r == CALC_VALUES) begin
      if (calc_values_in_one_pair_counter_r == 1) begin
        if (current_indices_in_order_r[0] != 0 && current_indices_in_order_r[1] != 0) begin
          calc_values_column_ind_r <= 0;
        end else if (current_indices_in_order_r[0] != 1 && current_indices_in_order_r[1] != 1) begin
          calc_values_column_ind_r <= 1;
        end else begin
          calc_values_column_ind_r <= 2;
        end
      end else begin
        if (calc_values_column_ind_r == current_indices_in_order_r[0] - 1 || calc_values_column_ind_r == current_indices_in_order_r[1] - 1) begin
          if (calc_values_column_ind_r == current_indices_in_order_r[0] - 2 || calc_values_column_ind_r == current_indices_in_order_r[1] - 2) begin
            calc_values_column_ind_r <= calc_values_column_ind_r + 3;
          end else begin
            calc_values_column_ind_r <= calc_values_column_ind_r + 2;
          end
        end else begin
          calc_values_column_ind_r <= calc_values_column_ind_r + 1;
        end
      end
    end

    if (rst == 1) begin
      calc_values_column_ind_r <= 0;
    end
  end



  /********* Delay push values to cordic flag ****/
  always_ff @(posedge clk) begin : calc_values_delay_push_values_to_cordic_p
    if (main_fsm_r == CALC_VALUES) begin
      calc_values_push_values_to_cordic_one_cycle_delayed_r <= calc_values_push_values_to_cordic_r;
    end

    if (rst == 1) begin
      calc_values_push_values_to_cordic_one_cycle_delayed_r <= 0;
    end
  end
  

  /******** Cordic receive counter ****/
  always_ff @(posedge clk) begin : calc_values_cordic_receive_counter_p
    if (main_fsm_r == CALC_VALUES) begin
      if (rotation_fifo_out_vld_i) begin
        if (calc_values_cordic_receive_counter_r < (JACOBI_N <<< 1) + 1) begin
          calc_values_cordic_receive_counter_r <= calc_values_cordic_receive_counter_r + 1;
        end else begin
          calc_values_cordic_receive_counter_r <= 0;
        end
      end
    end

    if (rst == 1) begin
      calc_values_cordic_receive_counter_r <= 0;
    end
  end

  /******** Get data from cordic ******/
  always_ff @(posedge clk) begin : calc_values_get_data_from_cordic_p
    if (main_fsm_r == CALC_VALUES) begin

      if (calc_values_push_values_to_cordic_one_cycle_delayed_r) begin
        rotation_in_dat_x_r <= ram_dout_a_i;
        rotation_in_dat_y_r <= ram_dout_b_i;
        rotation_in_dat_z_r <= angles_register_r[0];
        rotation_in_vld_r <= 1;
      end else begin
        rotation_in_vld_r <= 0;
      end

      rotation_fifo_out_rdy_r <= 1;
      if (rotation_fifo_out_vld_i) begin
        if (calc_values_cordic_receive_counter_r == 0) begin
          rotation_fifo_out_dat_x_temp_r <= rotation_fifo_out_dat_x_i;
          rotation_fifo_out_dat_y_temp_r <= rotation_fifo_out_dat_y_i;
        end else if (calc_values_cordic_receive_counter_r == 1) begin
          rotation_in_dat_x_r <= rotation_fifo_out_dat_x_temp_r;
          rotation_in_dat_y_r <= rotation_fifo_out_dat_x_i;
          rotation_in_dat_z_r <= angles_register_r[0];
          rotation_in_vld_r <= 1;

          rotation_fifo_out_dat_x_temp_r <= rotation_fifo_out_dat_y_i;
        end else if (calc_values_cordic_receive_counter_r == 2) begin
          rotation_in_dat_x_r <= rotation_fifo_out_dat_y_temp_r;
          rotation_in_dat_y_r <= rotation_fifo_out_dat_x_temp_r;
          rotation_in_dat_z_r <= angles_register_r[0];
          rotation_in_vld_r <= 1;

          rotation_fifo_out_dat_x_temp_r <= rotation_fifo_out_dat_x_i;
          rotation_fifo_out_dat_y_temp_r <= rotation_fifo_out_dat_y_i;
          calc_values_valid_r <= 1;
        end else if (calc_values_cordic_receive_counter_r < JACOBI_N <<< 1) begin
          rotation_fifo_out_dat_x_temp_r <= rotation_fifo_out_dat_x_i;
          rotation_fifo_out_dat_y_temp_r <= rotation_fifo_out_dat_y_i;
          calc_values_valid_r <= 1;
        end else if (calc_values_cordic_receive_counter_r == JACOBI_N <<< 1) begin
          rotation_fifo_out_dat_x_temp_r <= rotation_fifo_out_dat_x_i;
          calc_values_valid_r <= 0;
        end else if (calc_values_cordic_receive_counter_r == (JACOBI_N <<< 1) + 1) begin
          rotation_fifo_out_dat_y_temp_r <= rotation_fifo_out_dat_y_i;
          calc_values_valid_r <= 1;
        end
      end else begin
        calc_values_valid_r <= 0;
      end
    end

    if (rst == 1) begin
      rotation_fifo_out_rdy_r <= 0;
      calc_values_valid_r <= 0;
      rotation_in_vld_r <= 0;
    end
  end

  /******** Save data from cordic counter ******/
  always_ff @(posedge clk) begin : calc_values_save_data_from_cordic_counter_p
    if (main_fsm_r == CALC_VALUES) begin
      if (calc_values_valid_r) begin
        calc_values_save_data_from_cordic_counter_r <= calc_values_save_data_from_cordic_counter_r + 1;
      end else if (calc_values_save_data_from_cordic_counter_r == (JACOBI_N <<< 1) - 1) begin
        calc_values_save_data_from_cordic_counter_r <= 0;
      end
    end

    if (rst == 1) begin
      calc_values_save_data_from_cordic_counter_r <= 0;
    end
  end

  /******** Calc values write calc column indexes **/
  always_ff @(posedge clk) begin : calc_values_write_column_ind_p
    if (main_fsm_r == CALC_VALUES) begin
      if (!calc_values_valid_r) begin
        if (current_indices_in_order_r[0] != 0 && current_indices_in_order_r[1] != 0) begin
          calc_values_write_column_ind_r <= 0;
        end else if (current_indices_in_order_r[0] != 1 && current_indices_in_order_r[1] != 1) begin
          calc_values_write_column_ind_r <= 1;
        end else begin
          calc_values_write_column_ind_r <= 2;
        end
      end else begin
        if (calc_values_write_column_ind_r == current_indices_in_order_r[0] - 1 || calc_values_write_column_ind_r == current_indices_in_order_r[1] - 1) begin
          if (calc_values_write_column_ind_r == current_indices_in_order_r[0] - 2 || calc_values_write_column_ind_r == current_indices_in_order_r[1] - 2) begin
            calc_values_write_column_ind_r <= calc_values_write_column_ind_r + 3;
          end else begin
            calc_values_write_column_ind_r <= calc_values_write_column_ind_r + 2;
          end
        end else begin
          calc_values_write_column_ind_r <= calc_values_write_column_ind_r + 1;
        end
      end
    end

    if (rst == 1) begin
      calc_values_write_column_ind_r <= 0;
    end
  end

  always_ff @(posedge clk) begin : send_data_counter_p

    if (main_fsm_r == SEND_DATA && out_rdy_i == 1) begin
      if (send_data_counter_r == JACOBI_MEM_SIZE-1) begin
        send_data_counter_r <= 0;
      end else begin
        send_data_counter_r <= send_data_counter_r + 1;
      end
    end

    if (rst == 1) begin
      send_data_counter_r <= 0;
    end

  end

  /******** Calc values write calc column indexes **/
  always_ff @(posedge clk) begin : out_vld_control_p

    //Output valid must be 2 cycles delayed
    sender_vld_r <= sender_vld_s;
    out_vld_r <= sender_vld_r;
    
    //last as well
    last_r <= last_s;
    out_last_r <= last_r;

    if (rst == 1) begin
      sender_vld_r <= 0;
      out_vld_r <= 0;
      last_r <= 0;
      out_last_r <= 0;
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
  assign rotation_in_dat_x_o = rotation_in_dat_x_r;
  assign rotation_in_dat_y_o = rotation_in_dat_y_r;
  assign rotation_in_dat_z_o = rotation_in_dat_z_r;
  assign rotation_in_vld_o = rotation_in_vld_r;

  assign rotation_fifo_out_rdy_o = rotation_fifo_out_rdy_r;
  assign out_vld_o = out_vld_r;
  assign out_dat_o = ram_dout_a_i;
  assign out_last_o = out_last_r;

endmodule