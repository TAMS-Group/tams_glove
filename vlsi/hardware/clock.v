// 2021-2024 Philipp Ruppel

`include "utils.v"

module clock #(
  parameter CLKFREQ = 1000000
) (
  `make_mem_ports()
);

  reg [31:0] cycle_counter = 0;

  reg [31:0] microsecond_div = 0;
  reg [31:0] microsecond_counter = 0;

  reg [31:0] millisecond_div = 0;
  reg [31:0] millisecond_counter = 0;

  always @(posedge clk) begin

    cycle_counter <= cycle_counter + 1;

    microsecond_div <= microsecond_div + 1000000;
    if (microsecond_div >= CLKFREQ) begin
        microsecond_div <= microsecond_div + 1000000 - CLKFREQ;
        microsecond_counter <= microsecond_counter + 1;
    end

    millisecond_div <= millisecond_div + 1000;
    if (millisecond_div >= CLKFREQ) begin
        millisecond_div <= millisecond_div + 1000 - CLKFREQ;
        millisecond_counter <= millisecond_counter + 1;
    end

    `mem_begin();
    `reg_32_r(4*1, CLKFREQ);
    `reg_32_r(4*2, cycle_counter);
    `reg_32_r(4*3, microsecond_counter);
    `reg_32_r(4*4, millisecond_counter);

  end

endmodule
