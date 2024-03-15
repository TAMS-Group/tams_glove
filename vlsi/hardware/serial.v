// 2021-2024 Philipp Ruppel

`include "utils.v"

module serial (
    `make_mem_ports(),
    output reg pin
);

  reg [15:0] counter = 0;
  reg [15:0] div = 16'hffff;
  reg [15:0] data = 0;

  always @(posedge clk) begin

    counter <= counter + 1;

    if (counter >= div) begin
      counter <= 1;
      data[14:0] <= data[15:1];
    end

    pin <= data[0];

    `mem_begin();

    if (mem_valid && !mem_ready && mem_addr == 0 && mem_wstrb == 4'b1111) begin
      div <= mem_wdata[31:16];
      data <= mem_wdata[15:0];
      mem_ready <= 1;
      mem_error <= 0;
    end

  end

endmodule
