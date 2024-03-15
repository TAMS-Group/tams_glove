// 2023-2024 Philipp Ruppel

`include "async_fifo.v"
`include "packet_buffer.v"
`include "packet_prefixer.v"

module packet_framer #(
    parameter WORD_SIZE = 4,
    parameter INPUT_WORDS = 1,
    parameter OUTPUT_WORDS = 1,
    parameter HEADER_WORDS = 1,
    parameter PACKET_BUFFER_SIZE = 32,
    parameter FIFO_DEPTH = 8,
    parameter LENGTH_BITS = 4,
    parameter OUTPUT_BUFFER_SIZE = 4
) (
    input wire in_clk,
    output wire in_full,
    input wire in_shift,
    input wire [INPUT_WORDS*WORD_SIZE-1:0] in_data,
    input wire in_end,

    input wire out_clk,
    input wire out_pop,
    output wire out_nempty,
    output wire [OUTPUT_WORDS*WORD_SIZE-1:0] out_data,

    input wire [HEADER_WORDS*WORD_SIZE-1:0] header
);

    wire buf_out_pop;
    wire buf_out_nempty;
    wire [INPUT_WORDS*WORD_SIZE-1:0] buf_out_data;
    wire [LENGTH_BITS-1:0] buf_out_length;
    wire buf_out_start;

    packet_buffer #(
        .DATA_BITS(INPUT_WORDS*WORD_SIZE),
        .LENGTH_BITS(LENGTH_BITS),
        .BUFFER_SIZE(PACKET_BUFFER_SIZE)
    ) packet_buffer_inst ( 
        .clk(in_clk), 

        .in_full(in_full),
        .in_shift(in_shift), 
        .in_data(in_data), 
        .in_end(in_end),
        
        .out_pop(buf_out_pop),
        .out_nempty(buf_out_nempty), 
        .out_data(buf_out_data), 
        .out_length(buf_out_length), 
        .out_start(buf_out_start)
    );

    localparam FIFO_WIDTH = (1 + LENGTH_BITS + INPUT_WORDS * WORD_SIZE);

	wire fifo_in_shift;
	wire [FIFO_WIDTH-1:0] fifo_in_data = { buf_out_start, buf_out_length, buf_out_data };
	wire fifo_in_full;

	wire fifo_out_nempty;
	wire fifo_out_pop;
	wire [FIFO_WIDTH-1:0] fifo_out_data;

    async_fifo #(
        .WIDTH(FIFO_WIDTH),
        .DEPTH(FIFO_DEPTH)
    ) fifo_inst (
        .in_clk(in_clk),
        .in_shift(fifo_in_shift),
        .in_data(fifo_in_data),
        .in_full(fifo_in_full),

        .out_clk(out_clk),
        .out_pop(fifo_out_pop),
        .out_data(fifo_out_data),
        .out_nempty(fifo_out_nempty)
    );

    assign buf_out_pop = (!fifo_in_full && buf_out_nempty);
    assign fifo_in_shift = buf_out_pop;

    wire pref_in_full;
    wire pref_in_shift;
    wire [INPUT_WORDS*WORD_SIZE-1:0] pref_in_data;
    wire [HEADER_WORDS*WORD_SIZE-1:0] pref_in_prefix;
    wire pref_in_start;
    
    packet_prefixer #(
        .WORD_SIZE(WORD_SIZE),
        .INPUT_WORDS(INPUT_WORDS),
        .OUTPUT_WORDS(OUTPUT_WORDS),
        .PREFIX_WORDS(HEADER_WORDS),
        .BUFFER_SIZE(OUTPUT_BUFFER_SIZE)
    ) packet_prefixer_inst ( 
        .clk(out_clk), 

        .in_full(pref_in_full),
        .in_shift(pref_in_shift), 
        .in_data(pref_in_data), 
        .in_prefix(pref_in_prefix),
        .in_start(pref_in_start),

        .out_pop(out_pop),
        .out_nempty(out_nempty), 
        .out_data(out_data)
    );

    assign fifo_out_pop = (!pref_in_full && fifo_out_nempty);
    assign pref_in_shift = fifo_out_pop;

    assign pref_in_data = fifo_out_data[INPUT_WORDS*WORD_SIZE-1:0];
    assign pref_in_start = fifo_out_data[LENGTH_BITS+INPUT_WORDS*WORD_SIZE];
    assign pref_in_prefix = (header | fifo_out_data[LENGTH_BITS+INPUT_WORDS*WORD_SIZE-1:INPUT_WORDS*WORD_SIZE]);

endmodule