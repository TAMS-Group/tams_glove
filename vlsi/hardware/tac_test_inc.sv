// 2023-2024 Philipp Ruppel

module top #(
    parameter FREQ_CLOCK = 40000000,
    parameter FREQ_BASE = 1000,
    parameter FREQ_STEP = 50,
    parameter FREQ_SWAP = 10,
    parameter ADC_CHANNELS = 8,
    parameter DAC_CHANNELS = 8,
    parameter ADC_BITS = 16,
    parameter DAC_BITS = 8,
    parameter OUT_BITS = 32
) ( 
    input wire clk,
    output wire [DAC_BITS-1:0] dac_elements [DAC_CHANNELS-1:0],
    input wire [ADC_BITS-1:0] adc_elements [ADC_CHANNELS-1:0],
    output integer dac_channels,
    output integer adc_channels,
    output integer dac_bits,
    output integer adc_bits,
    output integer freq_clock,
    output integer freq_swap,
    output integer freq_base,
    output integer freq_step,
    input real tx_trace_voltages [DAC_CHANNELS-1:0],
    input real adc_voltages [ADC_CHANNELS-1:0],
    input real adc_feedback_currents [ADC_CHANNELS-1:0],
    output wire out_valid,
    output wire [$clog2(DAC_CHANNELS)-1:0] out_dac,
    output wire [$clog2(ADC_CHANNELS)-1:0] out_adc,
    output wire out_phase,
    output wire signed [OUT_BITS-1:0] out_data
); 

    assign dac_channels = DAC_CHANNELS;
    assign adc_channels = ADC_CHANNELS;
    assign dac_bits = DAC_BITS;
    assign adc_bits = ADC_BITS;
    assign freq_clock = FREQ_CLOCK;
    assign freq_swap = FREQ_SWAP;
    assign freq_base = FREQ_BASE;

    wire [ADC_BITS*ADC_CHANNELS-1:0] adc_data;
    wire [DAC_BITS*DAC_CHANNELS-1:0] dac_data;
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
        .out_valid(out_valid),
        .out_dac(out_dac),
        .out_adc(out_adc),
        .out_phase(out_phase),
        .out_data(out_data),
        .freq_swap(FREQ_SWAP),
        .freq_base(FREQ_BASE),
        .freq_step(FREQ_STEP)
    );

    integer i, j;
    genvar g;
    
    generate
        for (g = 0; g < DAC_CHANNELS; g = g + 1) begin
            assign dac_elements[g] = dac_data[(g+1)*DAC_BITS-1:g*DAC_BITS]; 
        end
    endgenerate

    generate
        for (g = 0; g < ADC_CHANNELS; g = g + 1) begin
            assign adc_data[(g+1)*ADC_BITS-1:g*ADC_BITS] = adc_elements[g];
        end
    endgenerate

endmodule