// 2021-2024 Philipp Ruppel

module dac #(
    parameter CHANNELS = 4
) (
  `make_mem_ports(),
  input wire [8*CHANNELS-1:0] data,
  output reg [CHANNELS-1:0] pins
);

  reg [7:0] accu [CHANNELS-1:0];
  reg [8*CHANNELS-1:0] data_buf;

  integer i;

  reg [15:0] clkdiv_counter = 0;
  reg [15:0] clkdiv_period = -1;

  always @(posedge clk) begin

    data_buf <= data;

    `mem_begin();
    `reg_32(4*1, clkdiv_period);

    if (clkdiv_counter == clkdiv_period) begin
      clkdiv_counter <= 0;
      for (i = 0; i < CHANNELS; i = i + 1) begin
          {pins[i],accu[i]} <= accu[i] + data_buf[i*8+7:i*8];
      end
    end else begin
      clkdiv_counter <= clkdiv_counter + 1;
    end

  end
endmodule
