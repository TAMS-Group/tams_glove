// 2021-2024 Philipp Ruppel

`include "utils.v"

module thermometer (
    `make_mem_ports()
);

    reg reg_startpulse = 0;
    reg dtr_startpulse = 0;
    wire [7:0] dtr_out;
    reg [7:0] reg_out = 0;

    DTR dtr (
        .STARTPULSE(dtr_startpulse),
        .DTROUT0(dtr_out[0]),
        .DTROUT1(dtr_out[1]),
        .DTROUT2(dtr_out[2]),
        .DTROUT3(dtr_out[3]),
        .DTROUT4(dtr_out[4]),
        .DTROUT5(dtr_out[5]),
        .DTROUT6(dtr_out[6]),
        .DTROUT7(dtr_out[7])
    );

    always @(posedge clk) begin
        dtr_startpulse <= reg_startpulse;
        reg_startpulse <= 0;
        reg_out <= dtr_out;
        `mem_begin();
        `reg_1_w(4*1, reg_startpulse);
        `reg_8_r(4*2, reg_out);
    end

endmodule
