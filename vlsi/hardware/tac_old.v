// 2021-2024 Philipp Ruppel

`include "utils.v"

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

    input wire [DAC_BITS:0] dac_scale,
    output wire [DAC_BITS*DAC_CHANNELS-1:0] dac_data,

    output reg out_valid,
    output reg [$clog2(DAC_CHANNELS)-1:0] out_dac,
    output reg [$clog2(ADC_CHANNELS)-1:0] out_adc,
    output reg out_phase,
    output reg [OUT_BITS-1:0] out_data
);

    integer i;
    genvar g;

    localparam TWO_PI = 6.28318530718;

    parameter WAVE_BITS = 8;
    parameter WAVE_SAMPLES = (1 << WAVE_BITS);

    localparam INPUT_CHANNEL_INDEX_BITS = $clog2(ADC_CHANNELS);
    localparam OUTPUT_CHANNEL_INDEX_BITS = $clog2(DAC_CHANNELS);

    reg [DAC_BITS-1:0] sin_table_u [WAVE_SAMPLES-1:0];
    initial begin
        for (i = 0; i < WAVE_SAMPLES; i = i + 1) begin
            sin_table_u[i] = $rtoi(($sin(i * TWO_PI / WAVE_SAMPLES) * 0.5 + 0.5) * ((1 << DAC_BITS) - 1));
        end
    end
    reg [DAC_BITS-1:0] sin_value_u = 0;

    reg signed [7:0] sin_table_s [WAVE_SAMPLES-1:0];
    initial begin
        for (i = 0; i < WAVE_SAMPLES; i = i + 1) begin
            sin_table_s[i] = $rtoi(($sin(i * TWO_PI / WAVE_SAMPLES) * -0.49 + 0.5) * 255);
        end
    end
    reg [7:0] sin_value_s = 0;

    reg [WAVE_BITS-1:0] phase_values [DAC_CHANNELS-1:0];

    reg [WAVE_BITS-1:0] current_phase_value;

    reg [WAVE_BITS-1:0] sin_table_s_index;
    reg signed [7:0] sin_table_s_value;

    reg [31:0] phase_counters [DAC_CHANNELS-1:0];
    reg [31:0] phase_counter_temp;

    reg [INPUT_CHANNEL_INDEX_BITS-1:0] input_index = 0;
    reg [OUTPUT_CHANNEL_INDEX_BITS-1:0] output_index = 0;

    reg [31:0] clock_multiplier = 1;

    reg [2:0] state = 0;

    reg [ADC_BITS*ADC_CHANNELS-1:0] adc_curr;
    wire [ADC_BITS-1:0] adc_curr_arr [ADC_CHANNELS-1:0];
    generate
        for (g = 0; g < ADC_CHANNELS; g = g + 1)
            assign adc_curr_arr[g] = adc_curr[(g+1)*ADC_BITS-1:g*ADC_BITS];
    endgenerate

    reg [ADC_BITS*ADC_CHANNELS-1:0] adc_prev;
    wire [ADC_BITS-1:0] adc_prev_arr [ADC_CHANNELS-1:0];
    generate
        for (g = 0; g < ADC_CHANNELS; g = g + 1)
            assign adc_prev_arr[g] = adc_prev[(g+1)*ADC_BITS-1:g*ADC_BITS];
    endgenerate

    reg [ADC_BITS-1:0] adc_curr_val;
    reg [ADC_BITS-1:0] adc_prev_val;

    reg [$clog2(ADC_CHANNELS*DAC_CHANNELS*2)-1:0] data_index = 0;

    parameter DATA_MEM_INDEX_BITS = $clog2(ADC_CHANNELS*DAC_CHANNELS*2);

    parameter DATA_MEM_SIZE = (1<<DATA_MEM_INDEX_BITS);

    reg [DAC_BITS-1:0] dac_data_arr [DAC_CHANNELS-1:0];
    generate
        for (g = 0; g < DAC_CHANNELS; g = g + 1)
            assign dac_data[(g+1)*DAC_BITS-1:g*DAC_BITS] = dac_data_arr[g];
    endgenerate

    reg [31:0] data_matrix [DATA_MEM_SIZE-1:0];
    reg signed [31:0] data_value;

    reg [31:0] data_matrix_temp;

    reg [31:0] swap_counter = 0;
    reg swap_flag = 0;

    reg quadrature_flag = 0;

    reg [31:0] clock = 0;

    always @(posedge clk) begin

        clock <= clock + 1;

        out_valid <= 0;

        state <= state + 1;
        case (state)

            0: begin
                data_index <= data_index + 1;
                if (quadrature_flag == 0) begin

                    output_index <= output_index + 1;

                    if (output_index == (DAC_CHANNELS - 1)) begin
                        output_index <= 0;

                        input_index <= input_index + 1;

                        if (input_index == (ADC_CHANNELS - 1)) begin
                            
                            input_index <= 0;
                            data_index <= 0;

                            adc_prev <= adc_curr;
                            adc_curr <= adc_data;

                            swap_counter <= swap_counter + (ADC_CHANNELS*DAC_CHANNELS*8*2*freq_swap);

                            if (swap_counter > FREQ_CLOCK) begin

                                swap_counter <= swap_counter + (ADC_CHANNELS*DAC_CHANNELS*8*2*freq_swap) - FREQ_CLOCK;

                                swap_flag <= 1;
                            end else begin
                                swap_flag <= 0;
                            end
                        end
                    end
                end
            end

            1: begin
                clock_multiplier <= ((freq_base) + (output_index * freq_step)) * (1*WAVE_SAMPLES*DAC_CHANNELS*8);
                current_phase_value <= phase_values[output_index];
                phase_counter_temp <= phase_counters[output_index];
                data_matrix_temp <= data_matrix[data_index];
            end

            2: begin
                sin_table_s_index <= (current_phase_value + (quadrature_flag ? (1 << (WAVE_BITS - 2)) : 0));
                data_value <= data_matrix_temp;
            end

            3: begin
                sin_table_s_value <= sin_table_s[sin_table_s_index];
                sin_value_u <= sin_table_u[current_phase_value];

                adc_curr_val <= adc_curr_arr[input_index];
                adc_prev_val <= adc_prev_arr[input_index];


                if (phase_counter_temp > FREQ_CLOCK) begin
                    current_phase_value <= current_phase_value + 1;
                end
            end

            4: begin
                if (phase_counter_temp > FREQ_CLOCK) begin
                    phase_counter_temp <= phase_counter_temp + clock_multiplier - FREQ_CLOCK;
                    phase_values[output_index] <= current_phase_value;
                end else begin
                    phase_counter_temp <= phase_counter_temp + clock_multiplier;
                end

                data_value <= data_value
                    + sin_table_s_value
                    * $signed(adc_curr_val - adc_prev_val);

                sin_value_u <= (({DAC_BITS'h0, sin_value_u} * dac_scale) >> DAC_BITS);
            end

            6: begin
                if (quadrature_flag == 0) dac_data_arr[output_index] <= sin_value_u;

                phase_counters[output_index] <= phase_counter_temp;

                data_matrix[data_index] <= (swap_flag ? 0 : data_value);
                if (swap_flag) begin
                    out_valid <= 1;
                    out_dac <= output_index;
                    out_adc <= input_index;
                    out_phase <= quadrature_flag;
                    out_data <= data_value;
                end

                quadrature_flag <= ~quadrature_flag;
            end

        endcase

    end

endmodule
