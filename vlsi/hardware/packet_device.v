// 2021-2024 Philipp Ruppel

`include "utils.v"

module packet_device #(
    parameter CHANNELS = 1
) (
    `make_mem_ports(),
    input wire [CHANNELS-1:0] out_pop,
    output reg [CHANNELS-1:0] out_nempty,
    output reg [63:0] out_data,
    output reg out_end,
    output reg [7:0] out_phase_shift,
);

    initial begin
        out_data = 0;
        out_end = 0;
        out_nempty = 0;
        out_phase_shift = 0;
    end

    always @(posedge clk) begin

        out_nempty <= (out_nempty & (~out_pop));

        `mem_begin();

        `reg_32_w(4 * 1, out_data[31:0]);
        `reg_32_w(4 * 2, out_data[63:32]);
        `reg_1_w(4 * 3, out_end);
        `reg_32(4 * 4, out_nempty);
        `reg_8_w(4 * 5, out_phase_shift);

    end
 
endmodule
