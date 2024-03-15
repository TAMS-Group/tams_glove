// 2023-2024 Philipp Ruppel

`include "ringbuffer.v"

module top ( );

    localparam DEPTH = 5;
    localparam WIDTH = 8;

    reg clk = 0;

    wire full;
    wire empty;
	wire in_shift;
	wire [WIDTH-1:0] in_data;
	wire out_pop;
	wire [WIDTH-1:0] out_data;
    ringbuffer #(
        .DEPTH(DEPTH),
        .WIDTH(WIDTH)
    ) ringbuffer_inst (
        .clk(clk),
        .full(full),
        .empty(empty),
        .in_shift(in_shift),
        .in_data(in_data),
        .out_pop(out_pop),
        .out_data(out_data)
    );

    reg [WIDTH-1:0] i_counter = 0;
    reg [WIDTH-1:0] o_counter = 0;

    reg i_rand = 0;
    reg o_rand = 0;

    assign in_shift = (!full && i_rand);
    assign in_data = (in_shift ? i_counter : 0);

    assign out_pop = (!empty && o_rand);
    
    always @(posedge clk) begin
        i_rand <= ($random % 5 == 0);
        o_rand <= ($random % 5 == 0);
        if (in_shift) begin
            $display("i %0d", i_counter);
            i_counter <= i_counter + 1;
        end
        if (out_pop) begin
            $display("o %0d %0d", o_counter, out_data);
            o_counter <= o_counter + 1;
        end
    end

    integer i;
    initial begin
        for (i = 0; i < 10000; i = i + 1) begin
            clk <= ~clk;
            #1000;
        end
    end

endmodule
