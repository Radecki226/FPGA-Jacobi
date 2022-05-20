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
  output reg [DATA_WIDTH-1:0] dout_a,

  //port B
  input                       clk_b,
  input                       en_b,
  input                       we_b,
  input      [ADDR_WIDTH-1:0] addr_b,
  input      [DATA_WIDTH-1:0] din_b,
  output reg [DATA_WIDTH-1:0] dout_b  
   
);
  
  reg [DATA_WIDTH-1:0] ram [MEM_SIZE-1:0]; 
  
  // port A logic
  always_ff @(posedge clk_a) begin
    if (en_a) begin
      if (we_a) begin
        ram[addr_a] <= din_a;
      end
      dout_a <= ram[addr_a];
    end
  end

  // port B logic
  always_ff @(posedge clk_b) begin
    if (en_b) begin
      if (we_b) begin
        ram[addr_b] <= din_b;
      end
      dout_b <= ram[addr_b];
    end
  end
 endmodule
