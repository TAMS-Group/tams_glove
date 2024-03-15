// 2023-2024 Philipp Ruppel

`include "packet_buffer.v"

module top ( );

    localparam DATA_BITS = 8;
    localparam LENGTH_BITS = 8;
    localparam BUFFER_SIZE = 16;

    reg clk = 0;

    wire in_full;
    wire in_shift;
    wire [DATA_BITS-1:0] in_data;
    wire in_end;

    localparam TEST_N = 20;
    reg [TEST_N-1:0] test_end = 'b10011010100;

    reg [31:0] test_i = 0;

    reg rand1 = 0;
    always @(posedge clk) begin
        rand1 <= ($random % 2 == 0);
    end

    assign in_shift = (!in_full && rand1);
    assign in_data = test_i;
    assign in_end = ((test_end >> test_i) & 1);

    always @(posedge clk) begin
        if (in_shift) begin
            test_i = ((test_i + 1) % TEST_N);
        end
    end

    wire out_pop;
    wire out_nempty;
    wire [DATA_BITS-1:0] out_data;
    wire [LENGTH_BITS-1:0] out_length;
    wire out_start;
    wire out_end;

    reg rand2 = 0;
    always @(posedge clk) begin
        rand2 <= ($random % 2 == 0);
    end

    assign out_pop = (out_nempty && rand2);

    always @(posedge clk) begin
        if (out_pop) begin
            if (out_start) $write("start-%0d", out_length);
            $write(" %0d", out_data);
            if (out_end) $write(" end\n");
        end
    end

    packet_buffer #(
        .DATA_BITS(DATA_BITS),
        .LENGTH_BITS(LENGTH_BITS),
        .BUFFER_SIZE(BUFFER_SIZE)
    ) packet_buffer_inst ( 
        .clk(clk), 
        .in_full(in_full),
        .in_shift(in_shift), 
        .in_data(in_data), 
        .in_end(in_end),
        .out_pop(out_pop),
        .out_nempty(out_nempty), 
        .out_data(out_data), 
        .out_length(out_length), 
        .out_start(out_start),
        .out_end(out_end)
    );

    integer i;
    initial begin
        for (i = 0; i < 1000; i = i + 1) begin
            clk <= ~clk;
            #100;
        end
    end

endmodule
