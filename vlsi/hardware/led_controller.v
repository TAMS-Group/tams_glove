// 2021-2024 Philipp Ruppel

`include "led_ws2812b.v"
`include "utils.v"

module led_controller #(
  parameter LEDCOUNT = 1,
  parameter CLKFREQ = 10000000
) (
  `make_mem_ports(),
  output wire led_pin,
);

  localparam DATABITS = LEDCOUNT * 24;

  reg [DATABITS-1:0] ws_color;
  ws2812b #(
    .DATABITS(DATABITS),
    .CLKFREQ(CLKFREQ)
  ) led_inst (
    .clk(clk),
    .color(ws_color),
    .pin(led_pin)
  );

  integer i;

  always @(posedge clk) begin

    `mem_begin();
    for (i = 0; i < LEDCOUNT; i = i + 1) begin
      `reg_32_w(32'h00000000+4*i, ws_color[24*i+23:24*i]);
    end

  end

endmodule
