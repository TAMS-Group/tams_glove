// 2021-2024 Philipp Ruppel

`include "utils.v"

module usb_rx (
    `make_mem_ports(),
    output wire in_pop,
    input wire in_nempty,
    input wire [31:0] in_data
);

    reg reg_read = 0;
    assign in_pop = (reg_read && in_nempty);
    always @(posedge clk) begin
        reg_read <= 0;
        `mem_begin();
        `reg_32_r(4 * 1, in_data);
        `reg_1_r(4 * 2, in_nempty);
        `reg_1_w(4 * 3, reg_read);
    end

endmodule
