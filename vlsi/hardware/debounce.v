// 2021-2024 Philipp Ruppel

module debounce #(
  parameter CYCLES = 1000000,
  parameter INITIAL = 0
) (
  input wire clk,
  input wire in,
  output wire out
);

  reg [$clog2(CYCLES)+1:0] counter = 0;
  reg in_buf = INITIAL;
  reg in_prev = INITIAL;
  reg out_buf = INITIAL;

  assign out = out_buf;

  always @(posedge clk) begin

    in_buf <= in;
    in_prev <= in_buf;

    if (in_buf != in_prev) begin
      counter <= 0;
    end else begin
      if (counter == CYCLES) begin
        out_buf <= in_buf;
      end else begin
        counter <= counter + 1;
      end
    end

  end

endmodule
