// 2023-2024 Philipp Ruppel

`include "packet_framer.v"

module top ( );

    localparam WORD_SIZE = 8;
    localparam INPUT_WORDS = 4;
    localparam OUTPUT_WORDS = 2;
    localparam HEADER_WORDS = 3;
    localparam LENGTH_BITS = 8;

    localparam FIFO_DEPTH = 4;

    localparam PACKET_BUFFER_SIZE = 64;
    localparam OUTPUT_BUFFER_SIZE = 10;

    integer i;

    reg in_clk = 0;
    initial begin
        for (i = 0; i < 50000; i = i + 1) begin
            in_clk <= ~in_clk;
            #10;
        end
        $write("\n");
    end
    reg out_clk = 0;
    initial begin
        for (i = 0; i < 1000000; i = i + 1) begin
            out_clk <= ~out_clk;
            #3;
        end
        $write("\n");
    end

    wire in_full;
    wire in_shift;
    wire [INPUT_WORDS*WORD_SIZE-1:0] in_data;
    wire in_end;

    localparam TEST_N = 20;
    reg [TEST_N-1:0] test_end = 'b0100110100100;

    reg [31:0] test_i = 0;

    reg in_rand = 0;
    always @(posedge in_clk)
        in_rand <= ($random % 3 == 0);

    reg out_rand = 0;
    always @(posedge out_clk)
        out_rand <= ($random % 3 == 0);

    assign in_shift = (!in_full && in_rand);
    assign in_data = test_i | 32'h10203040;
    assign in_end = ((test_end >> test_i) & 1);

    always @(posedge in_clk) begin
        if (in_shift) begin
            test_i = ((test_i + 1) % TEST_N);
        end
    end

    wire out_pop;
    wire out_nempty;
    wire [OUTPUT_WORDS*WORD_SIZE-1:0] out_data;

    assign out_pop = (out_nempty && out_rand);

    integer testbuf = 0;
    integer testcnt = 0;
    task test(input [7:0] data);
        begin
            testbuf = ((testbuf << 8) | data);
            if ((testbuf & 16'hffff) == 16'hDCBA) begin
                testcnt = 0;
                $write("\npacket[%3d]", ((testbuf >> 16) & 8'hff));
            end else begin
                if (testcnt > 2) begin
                    if (((testcnt - 3) % 4) == 0) $write(" ");
                    $write("%0h", ((testbuf >> 16) & 8'hff));
                end
            end
            testcnt += 1;
        end
    endtask

    always @(posedge out_clk) begin
        if (out_pop) begin
            test(out_data & 8'hff);
            test((out_data >> 8) & 8'hff);
        end
    end

    packet_framer #(
        .WORD_SIZE(WORD_SIZE),
        .INPUT_WORDS(INPUT_WORDS),
        .OUTPUT_WORDS(OUTPUT_WORDS),
        .HEADER_WORDS(HEADER_WORDS),
        .PACKET_BUFFER_SIZE(PACKET_BUFFER_SIZE),
        .FIFO_DEPTH(FIFO_DEPTH),
        .LENGTH_BITS(LENGTH_BITS),
        .OUTPUT_BUFFER_SIZE(OUTPUT_BUFFER_SIZE)
    ) packet_framer_inst ( 
        .in_clk(in_clk), 
        .in_full(in_full),
        .in_shift(in_shift), 
        .in_data(in_data), 
        .in_end(in_end),

        .out_clk(out_clk), 
        .out_pop(out_pop),
        .out_nempty(out_nempty), 
        .out_data(out_data),

        .header(24'hBADC00)
    );

endmodule
