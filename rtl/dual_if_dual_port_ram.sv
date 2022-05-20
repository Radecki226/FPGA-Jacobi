/**
 * Dual port RAM with multiplexer and two interfaces
*/
module dual_if_dual_port_ram 
#(
  parameter ADDR_WIDTH = 7,
  parameter MEM_SIZE = 128
  parameter DATA_WIDTH = 20
)
(
  input                   clk,
  input                   if_select,

  //Interface 0
  //port A
  input                   if0_en_a,
  input                   if0_we_a,
  input  [ADDR_WIDTH-1:0] if0_addr_a,
  input  [DATA_WIDTH-1:0] if0_din_a,
  output [DATA_WIDTH-1:0] if0_dout_a

  //port B
  input                   if0_en_b,
  input                   if0_we_b,
  input  [ADDR_WIDTH-1:0] if0_addr_b,
  input  [DATA_WIDTH-1:0] if0_din_b,
  output [DATA_WIDTH-1:0] if0_dout_b  

  //Interface 1
  //port A
  input                   if1_en_a,
  input                   if1_we_a,
  input  [ADDR_WIDTH-1:0] if1_addr_a,
  input  [DATA_WIDTH-1:0] if1_din_a,
  output [DATA_WIDTH-1:0] if1_dout_a

  //port B
  input                   if1_en_b,
  input                   if1_we_b,
  input  [ADDR_WIDTH-1:0] if1_addr_b,
  input  [DATA_WIDTH-1:0] if1_din_b,
  output [DATA_WIDTH-1:0] if1_dout_b  
)
  
  // memory signals
  reg                  en_a,
  reg                  we_a,
  reg [ADDR_WIDTH-1:0] addr_a,
  reg [DATA_WIDTH-1:0] din_a,
  reg [DATA_WIDTH-1:0] dout_a
  reg                  en_b,
  reg                  we_b,
  reg [ADDR_WIDTH-1:0] addr_b,
  reg [DATA_WIDTH-1:0] din_b,
  reg [DATA_WIDTH-1:0] dout_b                       

  dual_port_ram #(ADDR_WIDTH, MEM_SIZE, DATA_WIDTH) bram (

    //port A
    .clk_a(clk),
    .en_a(en_a),
    .we_a(we_a),
    .addr_a(addr_a),
    .din_a(din_a),
    .dout_a(dout_a),

    //port B
    .clk_a(clk),
    .en_a(en_b),
    .we_b(we_b),
    .addr_b(addr_b),
    .din_b(din_b),
    .dout_b(dout_b)
  );

  always_comb begin
    if (if_select) begin
      //port A
      en_a <= if1_en_a;
      we_a <= if1_we_a;
      addr_a <= if1_addr_a;
      din_a <= if1_din_a;
      dout_a <= if1_dout_a;
      //port B
      en_b <= if1_en_b;
      we_b <= if1_we_b;
      addr_b <= if1_addr_b;
      din_b <= if1_din_b;
      dout_b <= if1_dout_b;
    end else begin
      //port A
      en_a <= if0_en_a;
      we_a <= if0_we_a;
      addr_a <= if0_addr_a;
      din_a <= if0_din_a;
      dout_a <= if0_dout_a;
      //port B
      en_b <= if0_en_b;
      we_b <= if0_we_b;
      addr_b <= if0_addr_b;
      din_b <= i0_din_b;
      dout_b <= if0_dout_b;  
    end
  end
  
