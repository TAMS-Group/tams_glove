// 2023-2024 Philipp Ruppel

`include "ringbuffer.v"

module top ( );

    localparam DEPTH = 100;
    localparam WIDTH = 8;

    reg clk = 0;

    wire full;
    wire empty;
    wire half_full;

	reg in_shift;
	reg [WIDTH-1:0] in_data;

	wire out_pop;
	wire [WIDTH-1:0] out_data;

    ringbuffer #(
        .DEPTH(DEPTH),
        .WIDTH(WIDTH)
    ) ringbuffer_inst (
        .clk(clk),
        .full(full),
        .half_full(half_full),
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

    assign out_pop = (!empty && o_rand);

    reg [DEPTH:0] p_counter = 0;
    
    always @(posedge clk) begin

        in_data <= 0;
        in_shift <= 0;

        if (p_counter > 0) begin
            p_counter <= p_counter - 1;
            $display("i %0d", i_counter);
            i_counter <= i_counter + 1;
            in_data <= i_counter;
            in_shift <= 1;
        end else if (!half_full && ($random % 5 == 0)) begin
            p_counter <= DEPTH / 3;
        end

        o_rand <= ($random % 5 == 0);
        if (out_pop) begin
            $display("o %0d %0d", o_counter, out_data);
            o_counter <= o_counter + 1;
        end

    end

    integer i;
    initial begin
        for (i = 0; i < 100000; i = i + 1) begin
            clk <= ~clk;
            #1000;
        end
    end

endmodule
