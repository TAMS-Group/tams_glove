// 2023-2024 Philipp Ruppel

`include "../3rdparty/crossclkfifo/crossclkfifo.v"

module top ( );

    localparam WIDTH = 8;

    reg clk = 0;

	wire in_shift;
	wire [WIDTH-1:0] in_data;
	wire in_full;
	wire in_nempty;

	wire out_pop;
	wire [WIDTH-1:0] out_data;
	wire out_nempty;

    crossclkfifo fifo_inst (
        .in_clk(clk),
        .in_shift(in_shift),
        .in_data(in_data),
        .in_full(in_full),
        .in_nempty(in_nempty),
        .out_clk(clk),
        .out_pop(out_pop),
        .out_data(out_data),
        .out_nempty(out_nempty)
    );

    reg rand1 = 0;
    always @(posedge clk) begin
        rand1 <= ($random % 10 == 0);
    end

    reg [WIDTH-1:0] i_counter = 0;
    always @(posedge clk) begin
        if (in_shift) begin
            i_counter <= i_counter + 1;
        end
    end

    assign in_shift = (!in_full && rand1);
    assign in_data = i_counter;



    reg rand2 = 0;
    always @(posedge clk) begin
        rand2 <= ($random % 10 == 0);
    end

    assign out_pop = (out_nempty && rand2);

    reg [WIDTH-1:0] cntr = 0;
    always @(posedge clk) begin
        if (out_pop) begin
            $display("%3d %3d", cntr, out_data);
            cntr <= cntr + 1;
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
