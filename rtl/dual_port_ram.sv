/* Dual port ram with read-first based on xilinx synthesis guide:
Dual-Port Block RAM with Two Write Ports in Read First Mode Example 
(Verilog)
*/
module dual_port_ram 
#(
  parameter ADDR_WIDTH = 7,
  parameter MEM_SIZE = 128,
  parameter DATA_WIDTH = 20
)
(

  //port A
  input                       clk_a,
  input                       en_a,
  input                       we_a,
  input      [ADDR_WIDTH-1:0] addr_a,
  input      [DATA_WIDTH-1:0] din_a,
  output     [DATA_WIDTH-1:0] dout_a,

  //port B
  input                       clk_b,
  input                       en_b,
  input                       we_b,
  input      [ADDR_WIDTH-1:0] addr_b,
  input      [DATA_WIDTH-1:0] din_b,
  output     [DATA_WIDTH-1:0] dout_b  
   
);
  
  reg [DATA_WIDTH-1:0] ram [MEM_SIZE-1:0]; 
  reg [ADDR_WIDTH-1:0] addr_a_r;
  reg [ADDR_WIDTH-1:0] addr_b_r;
  reg [DATA_WIDTH-1:0] dat_a_r;
  reg [DATA_WIDTH-1:0] dat_b_r;
  
  // port A logic
  always_ff @(posedge clk_a) begin
    if (en_a) begin
      if (we_a) begin
        ram[addr_a] <= din_a;
      end
      addr_a_r <= addr_a;
    end
  end

  always_ff @(posedge clk_a) begin
    if (en_a) begin
      dat_a_r <= ram[addr_a_r];
    end
  end
  assign dout_a = dat_a_r;

  // port B logic
  always_ff @(posedge clk_b) begin
    if (en_b) begin
      if (we_b) begin
        ram[addr_b] <= din_b;
      end
      addr_b_r <= addr_b;
    end 
  end
  
  always_ff @(posedge clk_b) begin
    if (en_b) begin
      dat_b_r <= ram[addr_b_r];
    end
  end
  assign dout_b = dat_b_r;
 endmodule

