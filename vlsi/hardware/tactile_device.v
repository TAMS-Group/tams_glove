// 2021-2024 Philipp Ruppel

`include "utils.v"
`include "adc.v"
`include "shfloat.v"

module tactile_device #(
    parameter FREQ_CLOCK = 10000000,
    parameter ADC_CHANNELS = 16,
    parameter DAC_CHANNELS = 16,
    parameter ADC_BITS = 16,
    parameter DAC_BITS = 8,
    parameter OUT_BITS = 32
) (
    `make_mem_ports(),
    input wire [ADC_CHANNELS-1:0] pins_adc_in,
    output wire [ADC_CHANNELS-1:0] pins_adc_out,
    output reg [DAC_CHANNELS-1:0] pins_dac_out
);

    integer i;

    wire [ADC_BITS*ADC_CHANNELS-1:0] adc_data;
    adc #(
        .CHANNELS(ADC_CHANNELS),
        .ADC_BITS(ADC_BITS)
    ) tacadc_inst (
        .clk(clk),
        .pins_in(pins_adc_in),
        .pins_out(pins_adc_out),
        .data(adc_data)
    );

    reg [DAC_BITS*DAC_CHANNELS-1:0] dac_data;
    reg [DAC_BITS*DAC_CHANNELS-1:0] dac_data_buf;
    reg [DAC_BITS-1:0] dac_accu [DAC_CHANNELS-1:0];
    reg [DAC_CHANNELS-1:0] dac_temp;
    always @(posedge clk) begin
        dac_data_buf <= dac_data;
        for (i = 0; i < DAC_CHANNELS; i = i + 1) begin
            {dac_temp[i],dac_accu[i]} <= dac_accu[i] + dac_data_buf[i*8+7:i*8];
        end
        pins_dac_out <= dac_temp;
    end

    wire tac_out_valid;
    wire [$clog2(DAC_CHANNELS)-1:0] tac_out_dac;
    wire [$clog2(ADC_CHANNELS)-1:0] tac_out_adc;
    wire tac_out_phase;
    wire [OUT_BITS-1:0] tac_out_data;
    reg [31:0] tac_freq_swap = 10;
    reg [31:0] tac_freq_base = 1000;
    reg [31:0] tac_freq_step = 50;
    reg [DAC_BITS:0] tac_dac_scale = 256;
    tactile #(
        .FREQ_CLOCK(FREQ_CLOCK),
        .ADC_CHANNELS(ADC_CHANNELS),
        .DAC_CHANNELS(DAC_CHANNELS),
        .ADC_BITS(ADC_BITS),
        .DAC_BITS(DAC_BITS),
        .OUT_BITS(OUT_BITS)
    ) tac_inst (
        .clk(clk),
        .adc_data(adc_data),
        .dac_data(dac_data),
        .dac_scale(tac_dac_scale),
        .out_valid(tac_out_valid),
        .out_dac(tac_out_dac),
        .out_adc(tac_out_adc),
        .out_phase(tac_out_phase),
        .out_data(tac_out_data),
        .freq_swap(tac_freq_swap),
        .freq_base(tac_freq_base),
        .freq_step(tac_freq_step)
    );

    wire tac_reg_valid;
    wire tac_reg_phase;
    wire [OUT_BITS-1:0] tac_reg_data;
    wire [31:0] tac_reg_addr;
    always @(posedge clk) begin
        tac_reg_valid <= tac_out_valid;
        tac_reg_phase <= tac_out_phase;
        tac_reg_data <= tac_out_data;
        tac_reg_addr <= (tac_out_adc * DAC_CHANNELS + tac_out_dac);
    end

    reg [31:0] tac_quad_buf [ADC_CHANNELS*DAC_CHANNELS-1:0];
    reg [31:0] tac_buf_i = 0;
    reg [31:0] tac_buf_q = 0;
    reg tac_buf_valid = 0;
    reg [31:0] tac_buf_addr = 0;
    always @(posedge clk) begin
        tac_buf_valid <= 0;
        if (tac_reg_valid) begin
            if (tac_reg_phase) begin
                tac_quad_buf[tac_reg_addr] <= tac_reg_data;
            end else begin
                tac_buf_i <= tac_reg_data;
                tac_buf_q <= tac_quad_buf[tac_reg_addr];
                tac_buf_valid <= 1;
                tac_buf_addr <= tac_reg_addr;
            end
        end
    end

    wire [31:0] shf_out_index;
    wire [31:0] shf_out_pack;
    wire shf_out_strobe;
    shfloat shfloat_inst (
        .clk(clk),
        .in_index(tac_buf_addr),
        .in_value_i(tac_buf_i),
        .in_value_q(tac_buf_q),
        .in_strobe(tac_buf_valid),
        .out_index(shf_out_index),
        .out_pack(shf_out_pack),
        .out_strobe(shf_out_strobe),
    );
    

    reg [31:0] data_frame = 0;
    localparam DATA_ADDRESS_BITS = $clog2(ADC_CHANNELS*DAC_CHANNELS);
    localparam DATA_MEM_SIZE = (1<<DATA_ADDRESS_BITS);
    reg [OUT_BITS-1:0] data_matrix [DATA_MEM_SIZE-1:0];
    always @(posedge clk) begin
        if (shf_out_strobe) begin
            data_matrix[shf_out_index] <= shf_out_pack;
            if (shf_out_index == DAC_CHANNELS * ADC_CHANNELS - 1)
                data_frame <= data_frame + 1;
        end
    end

    always @(posedge clk) begin

        `mem_begin();

        `reg_32(4*2, tac_freq_swap);
        `reg_32(4*3, tac_freq_base);
        `reg_32(4*4, tac_freq_step);
        `reg_32(4*5, tac_dac_scale);
        `reg_32_r(4*7, data_frame);

        if (mem_valid && !mem_ready && (mem_addr[31:DATA_ADDRESS_BITS+2] == (32'h00030000 >> (DATA_ADDRESS_BITS+2)))) begin
            mem_rdata <= data_matrix[mem_addr[DATA_ADDRESS_BITS+2-1:2]];
            mem_ready <= 1;
            mem_error <= 0;
        end

    end

endmodule