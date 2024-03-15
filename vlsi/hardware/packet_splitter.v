// 2023-2024 Philipp Ruppel

module packet_splitter #(
    parameter SEGMENT_SIZE = 4,
    parameter SEGMENT_COUNT = 2
) (
    input wire clk,

    output wire in_full,
    input wire in_shift,
    input wire [SEGMENT_SIZE*SEGMENT_COUNT-1:0] in_data,
    input wire in_end,

    input wire out_pop,
    output wire out_nempty,
    output wire [SEGMENT_SIZE-1:0] out_data,
    output wire out_end
);

    localparam BUFFER_SEGMENTS = SEGMENT_COUNT * 2 + 2;

    reg [SEGMENT_SIZE-1:0] buffer_data [BUFFER_SEGMENTS-1:0];
    reg [BUFFER_SEGMENTS-1:0] buffer_end = 0;
    reg [$clog2(BUFFER_SEGMENTS+SEGMENT_COUNT)-1:0] buffer_fill = 0;

    assign in_full = (buffer_fill >= SEGMENT_COUNT);
    assign out_nempty = (buffer_fill > 0);

    assign out_data = buffer_data[0];
    assign out_end = buffer_end[0];

    integer i;

    wire in_shift_safe = (in_shift && !in_full);
    wire out_pop_safe = (out_pop && out_nempty);

    always @(posedge clk) begin
        if (out_pop_safe) begin
            for (i = 0; i < BUFFER_SEGMENTS - 1; i = i + 1) begin
                buffer_data[i] <= buffer_data[i + 1];
                buffer_end[i] <= buffer_end[i + 1];
            end
            buffer_data[BUFFER_SEGMENTS - 1] <= 0;
            buffer_end[BUFFER_SEGMENTS - 1] <= 0;
        end
        if (in_shift_safe) begin
            for (i = 0; i < SEGMENT_COUNT; i = i + 1) begin
                buffer_data[buffer_fill - out_pop_safe + i] <= (in_data >> (i * SEGMENT_SIZE));
            end
            buffer_end[buffer_fill - out_pop_safe + SEGMENT_COUNT - 1] <= in_end;
        end
        buffer_fill <= (buffer_fill + (in_shift_safe * SEGMENT_COUNT) - out_pop_safe);
    end

endmodule