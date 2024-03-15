// 2023-2024 Philipp Ruppel

`ifndef PACKET_FIFO_V
`define PACKET_FIFO_V

`include "async_fifo.v"

module packet_fifo #(
    parameter WORD_SIZE = 4,
    parameter FIFO_DEPTH = 8
) (
    input wire in_clk,
    output wire in_full,
    input wire in_shift,
    input wire [WORD_SIZE-1:0] in_data,
    input wire in_end,

    input wire out_clk,
    input wire out_pop,
    output wire out_nempty,
    output wire [WORD_SIZE-1:0] out_data,
    output wire out_end
);

    async_fifo #(
        .WIDTH(1 + WORD_SIZE),
        .DEPTH(FIFO_DEPTH)
    ) fifo_inst (
        .in_clk(in_clk),
        .in_shift(in_shift),
        .in_data({in_end, in_data}),
        .in_full(in_full),

        .out_clk(out_clk),
        .out_pop(out_pop),
        .out_data({out_end, out_data}),
        .out_nempty(out_nempty)
    );

endmodule

`endif