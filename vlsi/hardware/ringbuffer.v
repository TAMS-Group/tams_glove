// 2023-2024 Philipp Ruppel

`ifndef INC_RINGBUFFER_V
`define INC_RINGBUFFER_V

module ringbuffer #(
    parameter WIDTH = 1,
    parameter DEPTH = 2
) (
    input wire clk,

    output wire full,
    output wire half_full,
    output reg empty,

    input wire in_shift,
    input wire [WIDTH-1:0] in_data,

    input wire out_pop,
    output reg [WIDTH-1:0] out_data
);

    reg [WIDTH-1:0] buffer [DEPTH-1:0];

    reg [$clog2(DEPTH):0] write_ptr = 0;
    wire [$clog2(DEPTH):0] write_ptr_inc = ((write_ptr == (DEPTH - 1)) ? (0) : (write_ptr + 1));

    reg [$clog2(DEPTH):0] read_ptr = 0;
    wire [$clog2(DEPTH):0] read_ptr_inc = ((read_ptr == (DEPTH - 1)) ? (0) : (read_ptr + 1));
    
    wire [$clog2(DEPTH):0] next_read_ptr = (out_pop ? read_ptr_inc : read_ptr);

    assign full = (write_ptr_inc == read_ptr);

    assign half_full = (((write_ptr + DEPTH - read_ptr) % DEPTH) >= DEPTH / 2);

    initial begin
        empty = 1;
        out_data = 0;
    end

    always @(posedge clk) begin

        if (in_shift) begin
            buffer[write_ptr] <= in_data;
            write_ptr <= write_ptr_inc;
        end

        read_ptr <= next_read_ptr;
        out_data <= buffer[next_read_ptr];
        empty <= (next_read_ptr == write_ptr);

    end

endmodule

`endif