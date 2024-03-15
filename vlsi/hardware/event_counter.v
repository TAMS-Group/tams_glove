// 2021-2024 Philipp Ruppel

`include "utils.v"
`include "../3rdparty/gray.v"

module event_counter (
    `make_mem_ports(),
    input wire event_clock,
    input wire event_active
);

    reg [31:0] event_count = 0;
    reg [31:0] event_count_gray = 0;
    always @(posedge event_clock) begin
        if (event_active) begin
            event_count <= event_count + 1;
        end
        event_count_gray <= binary_to_gray(event_count);
    end

    reg [31:0] reg_count_gray = 0;
    reg [31:0] reg_count = 0;
    always @(posedge clk) begin

        reg_count_gray <= event_count_gray;
        reg_count <= gray_to_binary(reg_count_gray);

        `mem_begin();
        `reg_32_r(4 * 1, reg_count);

    end

endmodule
