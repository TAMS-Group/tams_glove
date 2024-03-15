// 2021-2024 Philipp Ruppel

`include "led_ws2812b.v"

module top ( );

  reg clk = 0;
  wire led;

  ws2812b ws_inst ( .clk(clk), .color(24'hff00ff), .pin(led) );

  integer i;

  initial begin
    for (i = 0; i < 1000; i = i + 1) begin
      $write(led);
      clk <= ~clk;
      #10;
    end
    $write("\n");
  end

endmodule
