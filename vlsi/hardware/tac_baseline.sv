// 2023-2024 Philipp Ruppel

module tactile #(
    parameter FREQ_CLOCK = 10000000,
    parameter ADC_CHANNELS = 16,
    parameter DAC_CHANNELS = 16,
    parameter ADC_BITS = 16,
    parameter DAC_BITS = 8,
    parameter OUT_BITS = 32
) (
    input wire clk,

    input wire [19:0] freq_base,
    input wire [9:0] freq_step,
    input wire [9:0] freq_swap,

    input wire [ADC_BITS*ADC_CHANNELS-1:0] adc_data,
    output reg [DAC_BITS*DAC_CHANNELS-1:0] dac_data,

    output wire out_valid,
    output wire [$clog2(DAC_CHANNELS)-1:0] out_dac,
    output wire [$clog2(ADC_CHANNELS)-1:0] out_adc,
    output wire out_phase,
    output wire [OUT_BITS-1:0] out_data
);

    typedef logic [ADC_BITS-1:0] ADC_WORD;
    typedef logic signed [31:0] INT32;

    integer t = 0;

    integer i, j, p;
    genvar g;

    real accu [ADC_CHANNELS*DAC_CHANNELS*2-1:0];

    initial begin
        for (i = 0; i < ADC_CHANNELS*DAC_CHANNELS*2; i = i + 1) begin
            accu[i] = 0;
        end
    end

    reg [ADC_BITS*ADC_CHANNELS-1:0] adc_data_prev;

    generate
        for (g = 0; g < DAC_CHANNELS; g = g + 1) begin
            always @(posedge clk) begin
                dac_data[(g+1)*DAC_BITS-1:g*DAC_BITS] <= (DAC_BITS'($rtoi(($sin(1.0 * t * (freq_base + g * freq_step) / FREQ_CLOCK * 2 * 3.14159265359) * 0.5 + 0.5) * ((1 << DAC_BITS) - 1))));
            end
        end
    endgenerate

    real accumulation_matrix [DAC_CHANNELS-1:0][ADC_CHANNELS-1:0][1:0];
    real output_matrix [DAC_CHANNELS-1:0][ADC_CHANNELS-1:0][1:0];

    integer t_frame = (t % (FREQ_CLOCK / INT32'(freq_swap)));

    wire swap = (t_frame == 0);

    logic signed [ADC_BITS-1:0] adc_curr, adc_prev, adc_diff;
    real adc_diff_real;

    real mod_wave, product;

    localparam pi = 3.14159265359;

    integer t_readout = t_frame - 10;

    /* verilator lint_off WIDTH */
    assign out_valid = ((t_readout >= 0) && (t_readout < DAC_CHANNELS * ADC_CHANNELS * 2));
    assign out_phase = t_readout % 2;
    assign out_dac = (t_readout / 2 % DAC_CHANNELS);
    assign out_adc = (t_readout / 2 / DAC_CHANNELS % ADC_CHANNELS);
    assign out_data = $rtoi(output_matrix[out_dac][out_adc][out_phase]);
    /* verilator lint_on WIDTH */

    always @(posedge clk) begin
        t <= t + 1;
        for (i = 0; i < DAC_CHANNELS; i = i + 1) begin
            for (j = 0; j < ADC_CHANNELS; j = j + 1) begin
                for (p = 0; p < 2; p = p + 1) begin
                    adc_curr = ADC_WORD'(adc_data >> j*ADC_BITS);
                    adc_prev = ADC_WORD'(adc_data_prev >> j*ADC_BITS);
                    adc_diff = adc_curr - adc_prev;
                    adc_diff_real = INT32'(adc_diff);
                    mod_wave = $sin(1.0 * t * (INT32'(freq_base) + i * INT32'(freq_step)) / FREQ_CLOCK * 2 * pi + p * pi * 0.5);
                    product = adc_diff_real * mod_wave;
                    if (swap) begin
                        accumulation_matrix[i][j][p] <= product;
                        output_matrix[i][j][p] <= accumulation_matrix[i][j][p];
                    end else begin
                        accumulation_matrix[i][j][p] <= accumulation_matrix[i][j][p] + product;
                    end
                end
            end
        end
        adc_data_prev <= adc_data;
    end

endmodule