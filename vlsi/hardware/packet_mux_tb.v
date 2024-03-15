// 2023-2024 Philipp Ruppel

`include "packet_mux.v"
`include "packet_fifo.v"

module top ( );

    localparam CHANNEL_COUNT = 4;
    localparam WORD_SIZE = 32;
    localparam SEGMENT_SIZE = 3;
    localparam INPUT_WORDS = 2;
    localparam OUTPUT_WORDS = 1;

    reg clk = 0;

    wire [CHANNEL_COUNT-1:0] mux_in_full;
    wire [CHANNEL_COUNT-1:0] mux_in_shift;
    
    wire [CHANNEL_COUNT-1:0] mux_in_end;
    wire [CHANNEL_COUNT*INPUT_WORDS*WORD_SIZE-1:0] mux_in_data;

    genvar g;
    generate
        for (g = 0; g < CHANNEL_COUNT; g = g + 1) begin
            reg fifo_in_rand = 0;
            reg [INPUT_WORDS*WORD_SIZE-1:0] counter = 2;
            assign mux_in_data[(g+1)*(INPUT_WORDS*WORD_SIZE)-1:g*(INPUT_WORDS*WORD_SIZE)] = counter;
            assign mux_in_shift[g] = (!mux_in_full[g] && fifo_in_rand); 
            assign mux_in_end[g] = ((counter % 5) == 0);
            always @(posedge clk) begin
                fifo_in_rand = (($random % 20) == 0);
                if (mux_in_shift[g]) begin
                    
                    counter <= counter + 1;
                end
            end
        end
    endgenerate

    wire mux_out_pop;
    wire mux_out_nempty;
    
    wire [OUTPUT_WORDS*WORD_SIZE-1:0] mux_out_data;
    
    packet_mux #(
        .CHANNEL_COUNT(CHANNEL_COUNT),
        .WORD_SIZE(WORD_SIZE),
        .SEGMENT_SIZE(SEGMENT_SIZE),
        .INPUT_WORDS(INPUT_WORDS),
        .OUTPUT_WORDS(OUTPUT_WORDS),
        .HEADER_TEMPLATE(32'h10000000),
        .HEADER_COUNT_SHIFT(0),
        .HEADER_CHANNEL_SHIFT(8),
        .HEADER_END_SHIFT(16)
    ) mux_inst (
        .clk(clk),

        .in_full(mux_in_full),
        .in_shift(mux_in_shift),
        .in_end(mux_in_end),
        .in_data(mux_in_data),

        .out_pop(mux_out_pop),
        .out_nempty(mux_out_nempty),
        .out_data(mux_out_data) 
        
    );

    reg mux_out_random = 0;
    assign mux_out_pop = (mux_out_nempty && mux_out_random);
    always @(posedge clk) begin
        mux_out_random <= ($random % 2);
        
        if (mux_out_pop) begin
            
            
            $display("%0h %d", mux_out_data, mux_out_data);
        end
    end

    integer clk_i;
    initial begin
        for (clk_i = 0; clk_i < 1000; clk_i = clk_i + 1) begin
            clk <= ~clk;
            #100;
        end
    end

endmodule
