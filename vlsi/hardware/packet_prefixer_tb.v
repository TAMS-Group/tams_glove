// 2023-2024 Philipp Ruppel

`include "packet_prefixer.v"

module top ( );

    localparam WORD_SIZE = 8;
    localparam INPUT_WORDS = 4;
    localparam OUTPUT_WORDS = 2;
    localparam PREFIX_WORDS = 3;
    localparam BUFFER_SIZE = 32;

    reg clk = 0;

    wire in_full;
    wire in_shift;
    wire [WORD_SIZE*INPUT_WORDS-1:0] in_data;
    wire in_end;
    wire [WORD_SIZE*PREFIX_WORDS-1:0] in_prefix = 24'hccbbaa;
    wire in_start;

    localparam TEST_N = 20;
    reg [TEST_N-1:0] test_start = 'b1001101001001;

    reg [31:0] test_i = 0;

    reg rand1 = 0;
    always @(posedge clk) begin
        rand1 <= ($random % 2 == 0);
    end

    assign in_shift = (!in_full && rand1);
    assign in_data = (test_i | 32'h10203040);
    
    assign in_start = ((test_start >> test_i) & 1);

    always @(posedge clk) begin
        if (in_shift) begin
            test_i <= ((test_i + 1) % TEST_N);
        end
    end

    wire out_pop;
    wire out_nempty;
    wire [WORD_SIZE*OUTPUT_WORDS-1:0] out_data;

    reg rand2 = 0;
    always @(posedge clk) begin
        rand2 <= ($random % 2 == 0);
    end

    assign out_pop = (out_nempty && rand2);

    always @(posedge clk) begin
        if (out_pop) begin
            $write(" %0h %0h", (out_data & 8'hff), ((out_data >> 8) & 8'hff));
        end
    end

    packet_prefixer #(
        .WORD_SIZE(WORD_SIZE),
        .INPUT_WORDS(INPUT_WORDS),
        .OUTPUT_WORDS(OUTPUT_WORDS),
        .PREFIX_WORDS(PREFIX_WORDS),
        .BUFFER_SIZE(BUFFER_SIZE)
    ) packet_prefixer_inst ( 
        .clk(clk), 
        .in_full(in_full),
        .in_shift(in_shift), 
        .in_data(in_data), 
        .in_prefix(in_prefix),
        .in_start(in_start),
        .out_pop(out_pop),
        .out_nempty(out_nempty), 
        .out_data(out_data)
    );

    integer i;
    initial begin
        for (i = 0; i < 1000; i = i + 1) begin
            clk <= ~clk;
            #100;
        end
    end

endmodule
