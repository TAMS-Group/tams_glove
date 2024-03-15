// 2021-2024 Philipp Ruppel

module gpio_core #(
    parameter CHANNELS = 4
) (
    `make_mem_ports(),
    input wire [CHANNELS-1:0] pins_i,
    output reg [CHANNELS-1:0] pins_o,
    output reg [CHANNELS-1:0] pins_t,
);

    reg [CHANNELS-1:0] buf_i;

    reg [31:0] reg_in;
    reg [31:0] reg_dir;
    reg [31:0] reg_out;

    reg spi_clock_out;
    reg spi_clock_dir;
    USRMCLK spi_clock ( .USRMCLKI(spi_clock_out), .USRMCLKTS(spi_clock_dir) );

    reg [$clog2(CHANNELS)-1:0] set_index = 0;
    reg [$clog2(CHANNELS)-1:0] get_index = 0;
    reg [$clog2(CHANNELS)-1:0] shift_index = 0;
    reg [7:0] shift_out_data = 0;
    reg shift_out_flag = 0;
    reg [7:0] shift_in_data = 0;
    reg shift_in_flag = 0;

    always @(posedge clk) begin

        buf_i <= pins_i;
        reg_in <= buf_i;

        pins_t <= ~reg_dir;
        pins_o <= reg_out;

        spi_clock_out <= ~reg_dir[31];
        spi_clock_dir <= reg_out[31];

        `mem_begin();
        `reg_32_r(32'h00000000+4*0, 12345);
        `reg_32_r(32'h00000000+4*1, CHANNELS);
        `reg_32_r(32'h00000000+4*2, reg_in);
        `reg_32(32'h00000000+4*3, reg_dir);
        `reg_32(32'h00000000+4*4, reg_out);

        `reg_8_w(32'h00000000+4*5, set_index);
        if (mem_valid && !mem_ready && mem_addr == 32'h00000000+4*6 && mem_wstrb == 4'b1111) begin
            reg_out[set_index] <= (mem_wdata != 0);
            mem_ready <= 1;
            mem_error <= 0;
        end

        `reg_8_w(32'h00000000+4*7, get_index);
        `reg_8_r(32'h00000000+4*8, reg_in[get_index]);

        `reg_8_w(32'h00000000+4*9, shift_out_data);
        shift_out_flag <= 0;
        if (shift_out_flag) begin
            shift_out_data <= (shift_out_data << 1);
            reg_out[shift_index] <= shift_out_data[7];
        end
        `reg_1_w(32'h00000000+4*10, shift_out_flag);

        `reg_8_w(32'h00000000+4*11, shift_index);

        shift_in_flag <= 0;
        if (shift_in_flag) begin
            shift_in_data <= ((shift_in_data << 1) | reg_in[shift_index]);
        end
        `reg_1_w(32'h00000000+4*12, shift_in_flag);
        `reg_8_r(32'h00000000+4*13, shift_in_data);

    end

endmodule
