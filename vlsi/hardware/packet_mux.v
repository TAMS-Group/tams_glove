// 2023-2024 Philipp Ruppel

`include "ringbuffer.v"

module packet_queue #(
    parameter WORD_SIZE = 8,
    parameter WORD_COUNT = 4
) (
    input wire clk,

    output wire in_full,
    input wire in_shift,
    input wire [WORD_SIZE-1:0] in_data,
    input wire in_end,

    input wire out_pop,
    output wire out_nempty,
    output wire out_end,
    output wire [WORD_SIZE-1:0] out_data,
    output wire [$clog2(WORD_COUNT):0] out_count
);

    reg [$clog2(WORD_COUNT):0] count = 0;
    reg [WORD_SIZE*WORD_COUNT-1:0] buffer_data = 0;
    reg [WORD_COUNT-1:0] buffer_end = 0;

    assign in_full = ((count == WORD_COUNT) || ((count > 0) && buffer_end[0]));
    assign out_nempty = (count > 0);
    assign out_count = count;
    assign out_data = (buffer_data >> ((count > 0) ? ((count - 1) * WORD_SIZE) : 13));
    
    assign out_end = buffer_end[0];

    always @(posedge clk) begin
        if (in_shift) begin
            buffer_data <= {buffer_data, in_data};
            buffer_end <= {buffer_end, in_end};
            
        end
        count <= (count + in_shift - out_pop);
    end

endmodule

module packet_mux #(
    parameter CHANNEL_COUNT = 4,
    parameter WORD_SIZE = 8,
    parameter INPUT_WORDS = 2,
    parameter OUTPUT_WORDS = 1,
    parameter SEGMENT_SIZE = 4,
    parameter BUFFER_WORDS = 4,
    parameter HEADER_WORDS = 1,
    parameter HEADER_TEMPLATE = 8'h80,
    parameter HEADER_COUNT_SHIFT = 0,
    parameter HEADER_CHANNEL_SHIFT = 4,
    parameter HEADER_END_SHIFT = 6
) (
    input wire clk,

    output wire [CHANNEL_COUNT-1:0] in_full,
    input wire [CHANNEL_COUNT-1:0] in_shift,
    input wire [CHANNEL_COUNT-1:0] in_end,
    input wire [CHANNEL_COUNT*INPUT_WORDS*WORD_SIZE-1:0] in_data,

    input wire out_pop,
    output wire out_nempty,
    output wire [OUTPUT_WORDS*WORD_SIZE-1:0] out_data
);

    wire [CHANNEL_COUNT-1:0] queue_out_pop;
    wire [CHANNEL_COUNT-1:0] queue_out_nempty;
    wire [CHANNEL_COUNT-1:0] queue_out_end;
    wire [INPUT_WORDS*WORD_SIZE-1:0] queue_out_data [CHANNEL_COUNT-1:0];
    wire [$clog2(SEGMENT_SIZE):0] queue_out_count [CHANNEL_COUNT-1:0];
    
    
    genvar g;
    generate
        for (g = 0; g < CHANNEL_COUNT; g = g + 1) begin
            
            packet_queue #(
                .WORD_SIZE(INPUT_WORDS*WORD_SIZE),
                .WORD_COUNT(SEGMENT_SIZE)
            ) pq (
                .clk(clk),
                .in_full(in_full[g]),
                .in_shift(in_shift[g]),
                .in_end(in_end[g]),
                .in_data(in_data[(g+1)*(INPUT_WORDS*WORD_SIZE)-1:g*(INPUT_WORDS*WORD_SIZE)]),
                .out_pop(queue_out_pop[g]),
                .out_nempty(queue_out_nempty[g]),
                .out_end(queue_out_end[g]),
                .out_data(queue_out_data[g]),
                .out_count(queue_out_count[g])
            );
        end
    endgenerate

    localparam CHANNEL_INDEX_WIDTH = $clog2(CHANNEL_COUNT);

    reg [CHANNEL_INDEX_WIDTH-1:0] channel_index = 0;

    reg [BUFFER_WORDS*WORD_SIZE-1:0] buffer_data = 0;
    reg [$clog2(BUFFER_WORDS)-1:0] buffer_top = 0;

    assign out_nempty = (buffer_top >= OUTPUT_WORDS);
    assign out_data = ((buffer_top >= OUTPUT_WORDS) ? (buffer_data >> ((buffer_top - 1) * WORD_SIZE)) : 0); 

    reg [$clog2(SEGMENT_SIZE):0] drain_count = 0;

    wire queue_pop = (queue_out_nempty[channel_index] && (buffer_top < BUFFER_WORDS - INPUT_WORDS - HEADER_WORDS - 1));
    assign queue_out_pop = (queue_pop ? (1 << channel_index) : 0); 

    wire [WORD_SIZE*HEADER_WORDS-1:0] header = HEADER_TEMPLATE | (queue_out_end[channel_index] << HEADER_END_SHIFT) | (channel_index << HEADER_CHANNEL_SHIFT) | (queue_out_count[channel_index] << HEADER_COUNT_SHIFT);

    wire [WORD_SIZE*INPUT_WORDS-1:0] queue_out_data_i = queue_out_data[channel_index];

    always @(posedge clk) begin
        if (queue_pop) begin
            if (drain_count > 0) begin
                drain_count <= drain_count - 1;
                if (drain_count == 1) channel_index <= ((channel_index + 1) % CHANNEL_COUNT);
                buffer_top <= buffer_top + INPUT_WORDS - (out_pop * OUTPUT_WORDS);
                buffer_data <= {buffer_data,queue_out_data_i};
                
            end else begin
                drain_count <= queue_out_count[channel_index] - 1;
                if (queue_out_count[channel_index] == 1) channel_index <= ((channel_index + 1) % CHANNEL_COUNT);
                buffer_top <= buffer_top + HEADER_WORDS + INPUT_WORDS - (out_pop * OUTPUT_WORDS);
                buffer_data <= {buffer_data,header,queue_out_data_i};
                
            end
        end else begin
            buffer_top <= buffer_top - (out_pop * OUTPUT_WORDS);
            if(!queue_out_nempty[channel_index])
                channel_index <= ((channel_index + 1) % CHANNEL_COUNT);
        end

    end

endmodule