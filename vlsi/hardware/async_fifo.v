// 2021-2024 Philipp Ruppel

`ifndef ASYNC_FIFO_V
`define ASYNC_FIFO_V

`include "../3rdparty/crossclkfifo/crossclkfifo.v"
`include "../3rdparty/afifo.v"

module async_fifo #(
	parameter WIDTH = 8,
	parameter DEPTH = 16
) (
	input wire in_clk,
	input wire in_shift,
	input wire [WIDTH-1:0] in_data,
	output wire in_full,
	output wire in_nempty,
	input wire out_clk,
	input wire out_pop,
	output wire [WIDTH-1:0] out_data,
	output wire out_nempty
);

    crossclkfifo #(
		.WIDTH(WIDTH),
		.DEPTH(DEPTH)
	) fifo_inst (
        .in_clk(in_clk),
        .in_shift(in_shift),
        .in_data(in_data),
        .in_full(in_full),
        .in_nempty(in_nempty),
        .out_clk(out_clk),
        .out_pop(out_pop),
        .out_data(out_data),
        .out_nempty(out_nempty)
    );

endmodule

`endif