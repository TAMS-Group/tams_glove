// 2023-2024 Philipp Ruppel

module packet_prefixer #(
    parameter WORD_SIZE = 4,
    parameter INPUT_WORDS = 1,
    parameter OUTPUT_WORDS = 1,
    parameter PREFIX_WORDS = 1,
    parameter BUFFER_SIZE = 32
) (
    input wire clk,

    output wire in_full,
    input wire in_shift,
    input wire [WORD_SIZE*INPUT_WORDS-1:0] in_data,
    input wire [WORD_SIZE*PREFIX_WORDS-1:0] in_prefix,
    input wire in_start,

    input wire out_pop,
    output wire out_nempty,
    output wire [WORD_SIZE*OUTPUT_WORDS-1:0] out_data
);

    integer i;
    genvar g;

    reg [$clog2(BUFFER_SIZE)-1:0] buffer_fill = 0;
    reg [WORD_SIZE-1:0] buffer_data [BUFFER_SIZE-1:0];

    assign in_full = (buffer_fill + INPUT_WORDS + PREFIX_WORDS >= BUFFER_SIZE);

    wire out_pop_safe = (out_pop && out_nempty);

    assign out_nempty = (buffer_fill >= OUTPUT_WORDS);
    generate
        for (g = 0; g < OUTPUT_WORDS; g = g + 1) begin
            assign out_data[(g + 1) * WORD_SIZE - 1 : g * WORD_SIZE] = buffer_data[g];
        end
    endgenerate

    always @(posedge clk) begin
        
        if (out_pop_safe) begin
            for (i = 0; i < BUFFER_SIZE - OUTPUT_WORDS; i = i + 1) begin
                buffer_data[i] <= buffer_data[i + OUTPUT_WORDS];
            end
            
        end
        if (in_shift) begin
            for (i = 0; i < INPUT_WORDS; i = i + 1) begin
                buffer_data[buffer_fill - (out_pop_safe ? OUTPUT_WORDS : 0) + ((in_shift && in_start) ? PREFIX_WORDS : 0) + i] <= (in_data >> (i * WORD_SIZE));
            end
            if (in_start) begin
                for (i = 0; i < PREFIX_WORDS; i = i + 1) begin
                    buffer_data[buffer_fill - (out_pop_safe ? OUTPUT_WORDS : 0) + i] <= (in_prefix >> (i * WORD_SIZE));
                end
            end
        end
        buffer_fill <= buffer_fill + (in_shift ? INPUT_WORDS : 0) + ((in_shift && in_start) ? PREFIX_WORDS : 0) - (out_pop_safe ? OUTPUT_WORDS : 0);
    end

endmodule