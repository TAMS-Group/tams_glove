// 2021-2024 Philipp Ruppel

`include "utils.v"

module tactile #(
    parameter INPUT_CHANNELS = 4,
    parameter OUTPUT_CHANNELS = 4,
    parameter ADC_BITS = 16
) (
    `make_mem_ports(),
    input wire [ADC_BITS*INPUT_CHANNELS-1:0] adc_data,
    output reg [8*OUTPUT_CHANNELS-1:0] dac_data
);

    integer i;

    localparam TWO_PI = 6.28318530718;

    parameter WAVE_BITS = 8;
    parameter WAVE_SAMPLES = (1 << WAVE_BITS);

    localparam INPUT_CHANNEL_BITS = $clog2(INPUT_CHANNELS);
    localparam OUTPUT_CHANNEL_BITS = $clog2(OUTPUT_CHANNELS);

    reg [7:0] sin_table_u [WAVE_SAMPLES-1:0];
    initial begin
        for (i = 0; i < WAVE_SAMPLES; i = i + 1) begin
            sin_table_u[i] = $sin(i * TWO_PI / WAVE_SAMPLES) * 35 + 36;
        end
    end
    reg [7:0] sin_value_u = 0;

    reg signed [7:0] sin_table_s [WAVE_SAMPLES-1:0];
    initial begin
        for (i = 0; i < WAVE_SAMPLES; i = i + 1) begin
            sin_table_s[i] = $sin(i * TWO_PI / WAVE_SAMPLES) * 125;
        end
    end
    reg [7:0] sin_value_s = 0;

    reg [WAVE_BITS-1:0] phase_values [OUTPUT_CHANNELS-1:0];

    reg [WAVE_BITS-1:0] wave_step = 0;

    reg [WAVE_BITS-1:0] current_phase_value;

    reg [WAVE_BITS-1:0] sin_table_s_index;
    reg signed [7:0] sin_table_s_value;

    reg [31:0] phase_counters [OUTPUT_CHANNELS-1:0];
    reg [31:0] phase_counter_temp;

    reg [INPUT_CHANNEL_BITS-1:0] input_index = 0;
    reg [OUTPUT_CHANNEL_BITS-1:0] output_index = 0;

    reg [31:0] clock_multipliers [OUTPUT_CHANNELS-1:0];

    reg [31:0] clock_multiplier = 1;
    reg [31:0] clock_divider = 1;

    reg [2:0] state = 0;

    reg [ADC_BITS*INPUT_CHANNELS-1:0] adc_curr;
    reg [ADC_BITS*INPUT_CHANNELS-1:0] adc_prev;

    reg [ADC_BITS-1:0] adc_curr_val;
    reg [ADC_BITS-1:0] adc_prev_val;

    reg [$clog2(INPUT_CHANNELS*OUTPUT_CHANNELS*2)-1:0] data_index = 0;

    parameter DATA_MEM_BITS = $clog2(INPUT_CHANNELS*OUTPUT_CHANNELS*2);

    parameter DATA_MEM_SIZE = (1<<DATA_MEM_BITS);

    reg [31:0] data_matrix [DATA_MEM_SIZE-1:0];
    reg signed [31:0] data_value;

    reg [31:0] data_matrix_temp;

    reg [31:0] data_mem [DATA_MEM_SIZE-1:0];

    reg [31:0] swap_counter = 0;
    reg [31:0] swap_frequency = 0;
    reg [31:0] swap_frame = 0;
    reg swap_flag = 0;

    reg quadrature_flag = 0;

    reg [31:0] clock = 0;

    reg [OUTPUT_CHANNELS-1:0] tx_mask = 0;

    reg [INPUT_CHANNEL_BITS:0] input_channels = 0;
    reg [OUTPUT_CHANNEL_BITS:0] output_channels = 0;

    always @(posedge clk) begin

        `mem_begin();

        `reg_32(32'h00010000+4*0, clock_divider);
        `reg_32_r(32'h00010000+4*1, INPUT_CHANNELS);
        `reg_32_r(32'h00010000+4*2, OUTPUT_CHANNELS);
        `reg_32_r(32'h00010000+4*3, swap_counter);
        `reg_32(32'h00010000+4*4, swap_frequency);
        `reg_32_r(32'h00010000+4*5, WAVE_SAMPLES);
        `reg_32_r(32'h00010000+4*7, clock);
        `reg_32(32'h00010000+4*8, tx_mask);
        `reg_32_r(32'h00010000+4*9, swap_frame);
        `reg_32(32'h00010000+4*10, wave_step);
        `reg_32(32'h00010000+4*20, input_channels);
        `reg_32(32'h00010000+4*21, output_channels);

        clock <= clock + 1;

        if (mem_valid && !mem_ready && (mem_addr[31:OUTPUT_CHANNEL_BITS+2] == (32'h00020000 >> (OUTPUT_CHANNEL_BITS+2)))) begin
            mem_rdata <= clock_multipliers[mem_addr[OUTPUT_CHANNEL_BITS+2-1:2]];
            if (mem_wstrb == 4'b1111) begin
                clock_multipliers[mem_addr[OUTPUT_CHANNEL_BITS+2-1:2]] <= mem_wdata;
            end
            mem_ready <= 1;
            mem_error <= 0;
        end

        if (mem_valid && !mem_ready && (mem_addr[31:DATA_MEM_BITS+2] == (32'h00030000 >> (DATA_MEM_BITS+2)))) begin
            mem_rdata <= data_mem[mem_addr[DATA_MEM_BITS+2-1:2]];
            mem_ready <= 1;
            mem_error <= 0;
        end

        if (mem_valid && !mem_ready && (mem_addr[31:OUTPUT_CHANNEL_BITS+2] == (32'h00040000 >> (OUTPUT_CHANNEL_BITS+2)))) begin
            mem_rdata <= phase_values[mem_addr[OUTPUT_CHANNEL_BITS+2-1:2]];
            mem_ready <= 1;
            mem_error <= 0;
        end

        state <= state + 1;
        case (state)

            0: begin
                data_index <= data_index + 1;
                if (quadrature_flag == 0) begin

                    output_index <= output_index + 1;
                    if (output_index == output_channels) begin
                        output_index <= 0;

                        input_index <= input_index + 1;
                        if (input_index == input_channels) begin
                            input_index <= 0;
                            data_index <= 0;

                            adc_prev <= adc_curr;
                            adc_curr <= adc_data;

                            swap_counter <= swap_counter + swap_frequency;
                            if (swap_counter > clock_divider) begin
                                swap_counter <= swap_counter + swap_frequency - clock_divider;
                                swap_flag <= 1;
                            end else begin
                                if (swap_flag) begin
                                  swap_frame <= swap_frame + 1;
                                end
                                swap_flag <= 0;
                            end
                        end
                    end
                end
            end

            1: begin

                clock_multiplier <= clock_multipliers[output_index];
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

                adc_curr_val <= adc_curr[ADC_BITS*input_index+ADC_BITS-1:ADC_BITS*input_index];
                adc_prev_val <= adc_prev[ADC_BITS*input_index+ADC_BITS-1:ADC_BITS*input_index];

                if (phase_counter_temp > clock_divider) begin
                    current_phase_value <= current_phase_value + wave_step;
                end
            end

            4: begin
                if (phase_counter_temp > clock_divider) begin
                    phase_counter_temp <= phase_counter_temp + clock_multiplier - clock_divider;
                    phase_values[output_index] <= current_phase_value;
                end else begin
                    phase_counter_temp <= phase_counter_temp + clock_multiplier;
                end

                data_value <= data_value
                    + sin_table_s_value
                    * $signed(adc_curr_val - adc_prev_val);

                if (!tx_mask[output_index])
                  sin_value_u <= 0;
            end

            6: begin
                dac_data[output_index*8+7:output_index*8] <= sin_value_u;
                phase_counters[output_index] <= phase_counter_temp;

                data_matrix[data_index] <= (swap_flag ? 0 : data_value);
                if (swap_flag) begin
                    data_mem[data_index] <= data_value;
                end

                quadrature_flag <= ~quadrature_flag;
            end

        endcase

    end

endmodule
