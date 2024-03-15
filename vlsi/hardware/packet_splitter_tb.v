// 2023-2024 Philipp Ruppel

`include "packet_splitter.v"

module top ( );

    localparam SEGMENT_SIZE = 8;
    localparam SEGMENT_COUNT = 4;

    reg clk = 0;

    wire in_full;
    reg in_shift = 0;
    reg [SEGMENT_SIZE*SEGMENT_COUNT-1:0] in_data = 0;
    reg in_end = 0;

    reg out_pop = 0;
    wire out_nempty;
    wire [SEGMENT_SIZE-1:0] out_data;
    wire out_end;

    packet_splitter #(
        .SEGMENT_SIZE(SEGMENT_SIZE),
        .SEGMENT_COUNT(SEGMENT_COUNT)
    ) inst (
        .clk(clk),

        .in_full(in_full),
        .in_shift(in_shift),
        .in_data(in_data),
        .in_end(in_end),

        .out_pop(out_pop),
        .out_nempty(out_nempty),
        .out_data(out_data),
        .out_end(out_end)
    );

    integer k;

    initial begin

        #10;

        for (k = 0; k < 100; k = k + 1) begin

            out_pop = out_nempty;
            if (out_pop) begin
                $display("output %0h %0b", out_data, out_end);
            end

            in_shift = 0;
            if ($random % 7 == 0) begin
                in_data = k | (k + 1 << 8) | (k + 2 << 16) | (k + 3 << 24);
                in_shift = !in_full;
                in_end = $random % 2;
            end

            #10;
            clk <= 1;
            #10;
            clk <= 0;

        end

    end

endmodule
