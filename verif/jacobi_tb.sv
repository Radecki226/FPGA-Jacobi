`timescale 1ns/1ps
`define N 8
`define IN_WORD_WIDTH 16
`define OUT_WORD_WIDTH 20

class jacobi_item;
  bit [`OUT_WORD_WIDTH-1:0] correct_data;
  bit [`OUT_WORD_WIDTH-1:0] obtained_data;
  
endclass

class driver;

  int fd;
  int n_matrix = 0;
  int n_entry = 0;


  bit [`IN_WORD_WIDTH-1:0] in_dat;

  virtual jacobi_if jif;

  string path;

  function new(string name);
    $sformat(path, "%s\\input_matrix.txt",name);
  endfunction
  
  task run();
    $display ("T=%0t [Driver] starting ...", $time);
    @ (posedge jif.clk);
    fd = $fopen(path, "r");
    //Iterate to the end of file
    while ($fscanf(fd, "%h", in_dat) == 1) begin     

      jif.in_vld <= 1;
      jif.in_dat <= in_dat;

      while(1) begin
        @ (posedge jif.clk);
        if (jif.in_rdy == 1) begin
          break;
        end
      end
      
      $display ("T=%0t [Driver] Driving item 0x%0h n_entry = %0d, n_matrix = %0d", $time, in_dat, n_entry, n_matrix); 

      if (n_entry == (`N/2)*(`N+1) - 1) begin
        n_entry = 0;
        n_matrix ++;
      end else begin
        n_entry++;
      end
        
    end

    jif.in_vld <= 0;
    $fclose(fd);
    
  endtask
endclass
    
  


class monitor;
  virtual jacobi_if jif;
  mailbox scb_mbx;
  bit [`OUT_WORD_WIDTH-1:0] out_dat;

  string path_eigenvectors, path_eigenvalues;

  int fd_v, fd_w;
  int n_iter = 0; //check whether eigenvectors or eigenvalues are being sent

  function new(string name);
    $sformat(path_eigenvectors, "%s\\eigenvectors_matrix.txt",name);
    $sformat(path_eigenvalues, "%s\\diagonal_matrix.txt", name);
  endfunction
  
  task run();
    $display ("T=%0t [Monitor] starting ...", $time);

    fd_v = $fopen(path_eigenvectors, "r");
    fd_w = $fopen(path_eigenvalues, "r");
    
    @(posedge jif.clk);
    jif.out_rdy <= 1;

    forever begin

      @(posedge jif.clk);
      if (jif.out_vld) begin
        jacobi_item item = new;

        /*Eigenvalues go first then eigenvectors we have to choose with which matrix we are comparing output data*/
        if (n_iter > (`N/2)*(`N+1) - 1) begin
          
          $fscanf(fd_w, "%h", item.correct_data);
          
          if (n_iter == (`N/2)*(`N+1) + `N*`N - 1) begin
            n_iter = 0;
          end else
            n_iter ++;
          end
        else begin
          $fscanf(fd_v, "%h", item.correct_data);
          n_iter++;
        end

        item.obtained_data = jif.out_dat;
   
        scb_mbx.put(item);               
      end
    end

  endtask
endclass

class scoreboard;
  mailbox scb_mbx;
  int n_matrix = 0;
  int n_entry = 0;
  int err_count = 0;
  string which = "eigenvalues";
  task run();
    forever begin
      jacobi_item item;
      scb_mbx.get(item);
      
      if (item.obtained_data != item.correct_data) begin
        err_count++;
        $display ("T=%0t [Scoreboard] ERROR! Mismatch matrix = %0d %s entry = %0d correct = 0x%0h obtained = 0x%0h",
                  $time, n_matrix, which, n_entry, item.correct_data, item.obtained_data);
      end else begin
        $display ("T=%0t [Scoreboard] PASS! matrix = %0d %s entry = %0d", $time, n_matrix, which, n_entry);
      end

      /*Count iterations to print correct entry/matrix*/
      if (which == "eigenvalues") begin
        if (n_entry == (`N/2)*(`N+1) - 1) begin
          n_entry = 0;
          which = "eigenvectors";
        end else begin
          n_entry++;
        end
      end else begin
        if (n_entry == `N*`N - 1) begin
          n_entry = 0;
          which = "eigenvalues";
          n_matrix++;
        end else begin
          n_entry++;
        end
      end
    end
  endtask
endclass

class env;
  driver 		  d0; 		// Driver handle
  monitor 		m0; 		// Monitor handle
  scoreboard	s0; 		// Scoreboard handle
  
  mailbox 	scb_mbx; 		// Connect MON -> SCB
  
  virtual jacobi_if jif; 	// Virtual interface handle
  
  function new(string name);
    d0 = new(name);
    m0 = new(name);
    s0 = new;

    scb_mbx = new();
    
    m0.scb_mbx = scb_mbx;
    s0.scb_mbx = scb_mbx;
    
  endfunction
  
  virtual task run();
    d0.jif = jif;
    m0.jif = jif;
    
    fork
      d0.run();
      m0.run();
      s0.run();
    join_any
  endtask
endclass

class test;
  env e0;
  virtual jacobi_if jif;
  
  function new(string name);
    e0 = new(name);
  endfunction
  
  task run();
    e0.jif = jif;
    e0.run();
  endtask

endclass

interface jacobi_if (input bit clk);

  logic 		              rst;

  logic [`IN_WORD_WIDTH-1:0]  in_dat;
  logic                   in_vld;
  logic                   in_rdy;

  logic [`OUT_WORD_WIDTH-1:0] out_dat;
  logic                   out_vld;
  logic                   out_rdy;

endinterface



module tb;

  reg clk;
  
  always #10 clk =~ clk;

  jacobi_if _if (clk);

  jacobi_top #(.N(`N),
               .IN_WORD_WIDTH(`IN_WORD_WIDTH), 
               .OUT_WORD_WIDTH(`OUT_WORD_WIDTH)) u0

              (.clk(clk),
              .rst(_if.rst),

              .in_dat_i(_if.in_dat),
              .in_vld_i(_if.in_vld),
              .in_rdy_o(_if.in_rdy),

              .out_dat_o(_if.out_dat),
              .out_vld_o(_if.out_vld),
              .out_rdy_i(_if.out_rdy));
  test t0;
  
  initial begin
    {clk, _if.rst} <= 1;
    
    // Apply reset and start stimulus
    #20 _if.rst <= 0;
    t0 = new("C:\\Users\\piotrek\\Desktop\\nauka\\semestr_8\\sdup\\FPGA-Jacobi\\model\\TV\\class test");
    
    t0.jif = _if;
    t0.run();

    #10000
    $display("[TB] Err count = %0d", t0.e0.s0.err_count);
    
    // Because multiple components and clock are running
    // in the background, we need to call $finish explicitly
    //#50 $finish;
  end
  
  // System tasks to dump VCD waveform file
  /*initial begin
    $dumpvars;
    $dumpfile ("dump.vcd");
  end*/
endmodule