import common::*;
module jacobi_addr_gen_lut(
  input unsigned  [JACOBI_LOG2_N-1:0]     n,
  input unsigned  [JACOBI_LOG2_N-1:0]     k,
  output unsigned [JACOBI_ADDR_WIDTH-1:0] addr
);
  
  reg unsigned [JACOBI_LOG2_N-1:0]     row; //smaller idx(should point to row)
  reg unsigned [JACOBI_LOG2_N-1:0]     column; //higher idx(should point to column)
  reg unsigned [JACOBI_ADDR_WIDTH-1:0] sum;

  //COMPARE
  always_comb begin
    if (n > k) begin
      row <= k;
      column <= n;
    end else begin
      row <= n;
      column <= k;
    end
  end
  
  // LUT
  always_comb begin
    case(row)
      0 : sum = 0;
      1 : sum = 1;
      2 : sum = 3;
      3 : sum = 6;
      4 : sum = 10;
      5 : sum = 15;
      6 : sum = 21;
      7 : sum = 28;
    endcase
  end

  //ADDR GEN
  assign addr = (row <<< 3) + column - sum; 


endmodule