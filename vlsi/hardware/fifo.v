// 2021-2024 Philipp Ruppel

`ifndef INC_FIFO_V
`define INC_FIFO_V

module simple_fifo #(
    parameter WIDTH = 1,
    parameter DEPTH = 2
) (
    input wire clk,

    input wire in_shift,
    input wire [WIDTH-1:0] in_data,

    input wire out_pop,
    output reg out_nempty,
    output reg [WIDTH-1:0] out_data
);

    reg [WIDTH-1:0] buffer [DEPTH-1:0];

    reg [$clog2(DEPTH)-1:0] in_ptr = 0;
    reg [$clog2(DEPTH)-1:0] out_ptr = 0;

    always @(posedge clk) begin

        out_data <= buffer[out_ptr];
        
        if (in_shift) begin
            buffer[in_ptr] <= in_data;
            in_ptr <= in_ptr + 1;
        end

        out_nempty <= (in_ptr != out_ptr);

        if ((in_ptr != out_ptr) && out_pop) begin
            out_ptr <= out_ptr + 1;
        end

    end

endmodule

`endif