// 2021-2024 Philipp Ruppel

`include "debounce.v"

module top ( );

  reg debounce_in = 0;
  wire debounce_out;

  reg clk = 0;

  debounce #( .CYCLES(4), .INITIAL(1) ) debounce_inst ( .clk(clk), .in(debounce_in), .out(debounce_out) );

  integer i;

  initial begin
    for (i = 0; i < 100; i = i + 1) begin
      $write(i / 10 % 2 ? 1'b1 : 1'b0);
    end
    $write("\n");
    for (i = 0; i < 100; i = i + 1) begin
      debounce_in <= (i / 10 % 2 ? 1'b1 : 1'b0);
      #10;
      $write(debounce_out);
      clk <= 0;
      #10;
      clk <= 1;
      #10;
    end
    $write("\n");
  end

endmodule
