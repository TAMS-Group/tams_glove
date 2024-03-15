// 2023-2024 Philipp Ruppel

module packet_buffer #(
    parameter DATA_BITS = 8,
    parameter BUFFER_SIZE = 32,
    parameter LENGTH_BITS = 8,
    parameter SAFETY_MARGIN = 5
) (
    input wire clk,

    output reg in_full,
    input wire in_shift,
    input wire [DATA_BITS-1:0] in_data,
    input wire in_end,

    input wire out_pop,
    output reg out_nempty,
    output reg [DATA_BITS-1:0] out_data,
    output reg [LENGTH_BITS-1:0] out_length,
    output reg out_start,
    output reg out_end
);

    reg [DATA_BITS-1:0] data_buffer [BUFFER_SIZE-1:0];
    reg [LENGTH_BITS-1:0] length_buffer [BUFFER_SIZE-1:0];
    reg [BUFFER_SIZE-1:0] start_buffer;
    reg [BUFFER_SIZE-1:0] end_buffer;

    localparam PTR_BITS = $clog2(BUFFER_SIZE)+2;

    reg [PTR_BITS-1:0] write_ptr = 1;
    reg [PTR_BITS-1:0] write_start_ptr = 1;
    reg [PTR_BITS-1:0] hold_ptr = 1;
    reg [PTR_BITS-1:0] next_read_ptr = 2;
    reg [PTR_BITS-1:0] read_ptr = 1;

    reg [LENGTH_BITS-1:0] length_counter = 1;

    integer i;

    initial begin
        out_nempty = 0;
        out_data = 0;
        out_length = 0;
        out_start = 0;
        out_end = 1;
    end

    always @(posedge clk) begin

        in_full <= ((((write_ptr + 3) % BUFFER_SIZE) == read_ptr) || (((write_ptr + 2) % BUFFER_SIZE) == read_ptr) || (((write_ptr + 1) % BUFFER_SIZE) == read_ptr));

        if (in_shift) begin

            data_buffer[write_ptr] <= in_data;
            start_buffer[write_ptr] <= (write_ptr == write_start_ptr);
            write_ptr <= (write_ptr + 1) % BUFFER_SIZE;
            length_counter <= length_counter + 1;

            if (in_end || (length_counter >= BUFFER_SIZE - 4)) begin
                end_buffer[write_ptr] <= 1;
                length_buffer[write_start_ptr] <= length_counter;
                hold_ptr <= write_ptr;
                write_start_ptr <= (write_ptr + 1) % BUFFER_SIZE;
                length_counter <= 1;
            end else begin
                end_buffer[write_ptr] <= 0;
            end

        end

        if (out_pop) begin
            out_nempty <= (next_read_ptr != hold_ptr);
            out_data <= data_buffer[next_read_ptr];
            out_length <= length_buffer[next_read_ptr];
            out_start <= start_buffer[next_read_ptr];
            out_end <= end_buffer[next_read_ptr];
            read_ptr <= next_read_ptr;
            next_read_ptr <= (next_read_ptr + 1) % BUFFER_SIZE;
        end else begin
            out_nempty <= (read_ptr != hold_ptr);
            out_data <= data_buffer[read_ptr];
            out_length <= length_buffer[read_ptr];
            out_start <= start_buffer[read_ptr];
            out_end <= end_buffer[read_ptr];
        end

    end

endmodule