// 2021-2024 Philipp Ruppel

`include "utils.v"

module packet_rx_device (
    `make_mem_ports(),
    output wire in_pop,
    input wire in_nempty,
    input wire [63:0] in_data,
    input wire in_end,
    output reg [7:0] out_phase_shift
);

    initial begin
        out_phase_shift = 0;
    end

    reg reg_read = 0;
    assign in_pop = (reg_read && in_nempty);

    always @(posedge clk) begin

        reg_read <= 0;

        `mem_begin();
        `reg_32_r(4 * 1, in_data[31:0]);
        `reg_32_r(4 * 2, in_data[63:32]);
        `reg_1_r(4 * 3, in_end);
        `reg_1_r(4 * 4, in_nempty);
        `reg_1_w(4 * 5, reg_read);
        `reg_8_w(4 * 6, out_phase_shift);

    end
 
endmodule
