`timescale 1ns/1ps
`define WIDTH 20

class cordic_item;
  bit [`WIDTH-1:0] x_correct;
  bit [`WIDTH-1:0] y_correct;
  bit [`WIDTH-1:0] z_correct;

  bit [`WIDTH-1:0] x_obtained;
  bit [`WIDTH-1:0] y_obtained;
  bit [`WIDTH-1:0] z_obtained;
  
endclass

class driver;

  int fd;
  int n_entry = 0;

  bit [`WIDTH-1:0] x_i;
  bit [`WIDTH-1:0] y_i;
  bit [`WIDTH-1:0] z_i;


  virtual cordic_if cif;

  string path;

  function new(string name);
    $sformat(path, "%s\\input_vectors.txt",name);
  endfunction
  
  task run();
    $display ("T=%0t [Driver] starting ...", $time);
    @ (posedge cif.clk);
    fd = $fopen(path, "r");
    //Iterate to the end of file
    while ($fscanf(fd, "%h %h %h", x_i, y_i, z_i) == 3) begin     

      cif.vld_i <= 1;
      cif.x_i   <= x_i;
      cif.y_i   <= y_i;
      cif.z_i   <= z_i;

      @ (posedge cif.clk);
     
      $display ("T=%0t [Driver] Driving item x = 0x%0h, y = 0x%0h, z = 0x%0h n_entry = %0d", $time, x_i, y_i, z_i, n_entry); 

      n_entry++;
        
    end

    cif.vld_i <= 0;
    $fclose(fd);
    
  endtask
endclass
    
  


class monitor;
  virtual cordic_if cif;
  mailbox scb_mbx;
  bit [`WIDTH-1:0] x_o;
  bit [`WIDTH-1:0] y_o;
  bit [`WIDTH-1:0] z_o;

  string path;

  int fd;

  function new(string name);
    $sformat(path, "%s\\output_vectors.txt",name);
  endfunction
  
  task run();
    $display ("T=%0t [Monitor] starting ...", $time);

    fd = $fopen(path, "r");

    forever begin

      @(posedge cif.clk);
      if (cif.vld_o) begin
        cordic_item item = new;

        $fscanf(fd, "%h %h %h", x_o, y_o, z_o);

        item.x_correct = x_o;
        item.y_correct = y_o;
        item.z_correct = z_o;

        item.x_obtained = cif.x_o;
        item.y_obtained = cif.y_o;
        item.z_obtained = cif.z_o;
   
        scb_mbx.put(item);               
      end
    end

  endtask
endclass

class scoreboard;
  mailbox scb_mbx;
  int n_entry = 0;
  int err_count = 0;
  task run();
    forever begin
      cordic_item item;
      scb_mbx.get(item);
      
      if (item.x_correct != item.x_obtained || item.y_correct != item.y_obtained  || item.z_correct != item.z_obtained ) begin
        err_count++;
        $display ("T=%0t [Scoreboard] ERROR! Mismatch entry = %0d\ncorrect:  x = 0x%0h y = 0x%0h z = 0x%0h\nobtained: x = 0x%0h y = 0x%0h z = 0x%0h" ,
                  $time, n_entry, item.x_correct, item.y_correct, item.z_correct, item.x_obtained, item.y_obtained, item.z_obtained);
      end else begin
        $display ("T=%0t [Scoreboard] PASS! entry = %0d", $time, n_entry);
      end

      /*Count iterations to print correct entry/matrix*/
      n_entry++;
    end
  endtask
endclass

class env;
  driver 		  d0; 		// Driver handle
  monitor 		m0; 		// Monitor handle
  scoreboard	s0; 		// Scoreboard handle
  
  mailbox 	scb_mbx; 		// Connect MON -> SCB
  
  virtual cordic_if cif; 	// Virtual interface handle
  
  function new(string name);
    d0 = new(name);
    m0 = new(name);
    s0 = new;

    scb_mbx = new();
    
    m0.scb_mbx = scb_mbx;
    s0.scb_mbx = scb_mbx;
    
  endfunction
  
  virtual task run();
    d0.cif = cif;
    m0.cif = cif;
    
    fork
      d0.run();
      m0.run();
      s0.run();
    join_any
  endtask
endclass

class test;
  env e0;
  virtual cordic_if cif;
  
  function new(string name);
    e0 = new(name);
  endfunction
  
  task run();
    e0.cif = cif;
    e0.run();
  endtask

endclass

interface cordic_if (input bit clk);

  logic 		          rst;

  logic [`WIDTH-1:0]  x_i;
  logic [`WIDTH-1:0]  y_i;
  logic [`WIDTH-1:0]  z_i;
  logic               vld_i;

  logic [`WIDTH-1:0]  x_o;
  logic [`WIDTH-1:0]  y_o;
  logic [`WIDTH-1:0]  z_o;
  logic               vld_o;

endinterface



module tb;

  reg clk;
  
  always #10 clk =~ clk;

  cordic_if _if (clk);

`define MODE "rotation"
  cordic #(
    .MODE(`MODE)
  ) u0 (
    .clk(clk),
    .rst(_if.rst),

    .x_i(_if.x_i),
    .y_i(_if.y_i),
    .z_i(_if.z_i),
    .vld_i(_if.vld_i),

    .x_o(_if.x_o),
    .y_o(_if.y_o),
    .z_o(_if.z_o),
    .vld_o(_if.vld_o)
  );
  test t0;
  
  initial begin
    {clk, _if.rst} <= 1;
    
    // Apply reset and start stimulus
    #20 _if.rst <= 0;
    t0 = new("C:\\Users\\piotrek\\Desktop\\nauka\\semestr_8\\sdup\\FPGA-Jacobi\\model\\TV\\cordic_rotation_test");
    
    t0.cif = _if;
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