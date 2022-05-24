`define MAX_WIDTH 64
package common;
  function [`MAX_WIDTH-1:0] fxp_round;
    input signed [`MAX_WIDTH-1:0] in_dat;
    input integer N;
    begin 
      fxp_round = (in_dat + (1 <<< N-1)) >>> N;
    end
  endfunction
endpackage