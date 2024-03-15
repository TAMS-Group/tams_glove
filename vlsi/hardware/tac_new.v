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

    initial out_valid = 0;

    localparam TWO_PI = 6.28318530718;

    localparam ADC_CHANNEL_INDEX_BITS = $clog2(ADC_CHANNELS);
    localparam DAC_CHANNEL_INDEX_BITS = $clog2(DAC_CHANNELS);

    integer i;
    genvar g;

    reg [ADC_CHANNEL_INDEX_BITS-1:0] seq_a_adc_channel = 0;
    reg [DAC_CHANNEL_INDEX_BITS-1:0] seq_a_dac_channel = 0;
    always @(posedge clk) begin
        if (seq_a_dac_channel == (DAC_CHANNELS - 1)) begin
            seq_a_dac_channel <= 0;
            if (seq_a_adc_channel == (ADC_CHANNELS - 1)) begin
                seq_a_adc_channel <= 0;
            end else begin
                seq_a_adc_channel <= seq_a_adc_channel + 1;
            end
        end else begin
            seq_a_dac_channel <= seq_a_dac_channel + 1;
        end
    end
    
    localparam SEQ_PHASE_BITS = 40;
    localparam SEQ_PHASE_STEP = $rtoi($pow(2.0, SEQ_PHASE_BITS) * DAC_CHANNELS / FREQ_CLOCK);
    reg [SEQ_PHASE_BITS-1:0] seq_phases [DAC_CHANNELS-1:0];
    initial begin
        for (i = 0; i < DAC_CHANNELS; i = i + 1) begin
            seq_phases[i] = 0;
        end
    end

    reg [ADC_CHANNEL_INDEX_BITS-1:0] seq_b_adc_channel = 0;
    reg [DAC_CHANNEL_INDEX_BITS-1:0] seq_b_dac_channel = 0;
    reg [SEQ_PHASE_BITS-1:0] seq_b_phase = 0;
    reg [SEQ_PHASE_BITS-1:0] seq_b_step = 0;
    always @(posedge clk) begin
        seq_b_adc_channel <= seq_a_adc_channel;
        seq_b_dac_channel <= seq_a_dac_channel;
        seq_b_phase <= seq_phases[seq_a_dac_channel];

        
        seq_b_step <= SEQ_PHASE_STEP * ((freq_base) + seq_a_dac_channel * freq_step);

    end

    reg [ADC_CHANNEL_INDEX_BITS-1:0] seq_out_adc_channel = 0;
    reg [DAC_CHANNEL_INDEX_BITS-1:0] seq_out_dac_channel = 0;
    reg [SEQ_PHASE_BITS-1:0] seq_out_phase = 0;
    always @(posedge clk) begin
        seq_out_adc_channel <= seq_b_adc_channel;
        seq_out_dac_channel <= seq_b_dac_channel;
        seq_out_phase <= seq_b_phase + seq_b_step;
    end

    always @(posedge clk) begin
        seq_phases[seq_out_dac_channel] <= seq_out_phase;
    end

    localparam EX_WAVE_INDEX_BITS = 8;
    localparam EX_WAVE_SAMPLES = (1 << EX_WAVE_INDEX_BITS);
    reg [DAC_BITS-1:0] ex_wave_table [EX_WAVE_SAMPLES-1:0];
    initial begin
        for (i = 0; i < EX_WAVE_SAMPLES; i = i + 1) begin
            ex_wave_table[i] = ($rtoi(($sin(i * TWO_PI / EX_WAVE_SAMPLES) * 0.5 + 0.5) * ((1 << DAC_BITS) - 1)));
        end
    end

    reg [DAC_BITS-1:0] dac_data_arr [DAC_CHANNELS-1:0];
    generate
        for (g = 0; g < DAC_CHANNELS; g = g + 1)
            assign dac_data[(g+1)*DAC_BITS-1:g*DAC_BITS] = dac_data_arr[g];
    endgenerate

    reg [DAC_CHANNEL_INDEX_BITS-1:0] ex_a_channel = 0;
    reg [DAC_BITS*2-1:0] ex_a_sample = 0;
    always @(posedge clk) begin
        ex_a_channel <= seq_out_dac_channel;
        ex_a_sample[DAC_BITS-1:0] <= ex_wave_table[seq_out_phase[SEQ_PHASE_BITS-1:SEQ_PHASE_BITS-EX_WAVE_INDEX_BITS]];
    end

    always @(posedge clk) begin
        dac_data_arr[ex_a_channel] <= ((ex_a_sample * dac_scale) >> DAC_BITS);
    end

    reg signed [ADC_BITS-1:0] adc_reg [ADC_CHANNELS-1:0];
    generate
        for (g = 0; g < ADC_CHANNELS; g = g + 1)
            always @(posedge clk) adc_reg[g] <= adc_data[(g+1)*ADC_BITS-1:g*ADC_BITS];
    endgenerate

    reg signed [ADC_BITS-1:0] adc_curr [ADC_CHANNELS-1:0];
    reg signed [ADC_BITS-1:0] adc_prev [ADC_CHANNELS-1:0];

    wire signed [ADC_BITS-1:0] adc_curr_seq_out_adc_channel = adc_curr[seq_out_adc_channel];
    wire signed [ADC_BITS-1:0] adc_prev_seq_out_adc_channel = adc_prev[seq_out_adc_channel];

    reg [ADC_CHANNEL_INDEX_BITS-1:0] adc_a_adc_channel = 0;
    reg [DAC_CHANNEL_INDEX_BITS-1:0] adc_a_dac_channel = 0;
    reg [SEQ_PHASE_BITS-1:0] adc_a_phase = 0;
    reg signed [ADC_BITS-1:0] adc_a_adc_curr = 0;
    reg signed [ADC_BITS-1:0] adc_a_adc_prev = 0;
    always @(posedge clk) begin
        if (seq_out_dac_channel == 0) begin
            adc_prev[seq_out_adc_channel] <= adc_curr_seq_out_adc_channel;
            adc_curr[seq_out_adc_channel] <= adc_reg[seq_out_adc_channel];
        end
        adc_a_adc_channel <= seq_out_adc_channel;
        adc_a_dac_channel <= seq_out_dac_channel;
        adc_a_phase <= seq_out_phase;
        adc_a_adc_curr <= adc_curr_seq_out_adc_channel;
        adc_a_adc_prev <= adc_prev_seq_out_adc_channel;
    end

    reg [ADC_CHANNEL_INDEX_BITS-1:0] adc_out_adc_channel = 0;
    reg [DAC_CHANNEL_INDEX_BITS-1:0] adc_out_dac_channel = 0;
    reg [SEQ_PHASE_BITS-1:0] adc_out_phase = 0;
    reg signed [ADC_BITS-1:0] adc_out_adc_value = 0;
    always @(posedge clk) begin
        adc_out_adc_channel <= adc_a_adc_channel;
        adc_out_dac_channel <= adc_a_dac_channel;
        adc_out_phase <= adc_a_phase;
        adc_out_adc_value <= adc_a_adc_curr - adc_a_adc_prev;
    end

    localparam CORR_WAVE_INDEX_BITS = 8;
    localparam CORR_WAVE_SAMPLES = (1 << CORR_WAVE_INDEX_BITS);
    localparam CORR_WAVE_BITS = 8;
    reg signed [CORR_WAVE_BITS-1:0] corr_wave_table_i [CORR_WAVE_SAMPLES-1:0];
    reg signed [CORR_WAVE_BITS-1:0] corr_wave_table_q [CORR_WAVE_SAMPLES-1:0];
    initial begin
        for (i = 0; i < CORR_WAVE_SAMPLES; i = i + 1) begin
            corr_wave_table_i[i] = ($rtoi(($sin(i * TWO_PI / CORR_WAVE_SAMPLES) * -0.49 + 0.5) * ((1 << CORR_WAVE_BITS) - 1)));
            corr_wave_table_q[i] = ($rtoi(($cos(i * TWO_PI / CORR_WAVE_SAMPLES) * -0.49 + 0.5) * ((1 << CORR_WAVE_BITS) - 1)));
        end
    end

    localparam CORR_ACCU_BITS = 32;
    localparam CORR_MAT_INDEX_BITS = $clog2(DAC_CHANNELS*ADC_CHANNELS);
    reg signed [CORR_ACCU_BITS-1:0] corr_accu_mat_i [DAC_CHANNELS*ADC_CHANNELS-1:0];
    reg signed [CORR_ACCU_BITS-1:0] corr_accu_mat_q [DAC_CHANNELS*ADC_CHANNELS-1:0];

    reg [ADC_CHANNEL_INDEX_BITS-1:0] corr_a_adc_channel = 0;
    reg [DAC_CHANNEL_INDEX_BITS-1:0] corr_a_dac_channel = 0;
    reg signed [CORR_WAVE_BITS-1:0] corr_a_wave_i = 0;
    reg signed [CORR_WAVE_BITS-1:0] corr_a_wave_q = 0;
    reg signed [ADC_BITS-1:0] corr_a_adc_value = 0;
    reg [CORR_MAT_INDEX_BITS-1:0] corr_a_mat_index = 0;
    reg [31:0] corr_a_swap_counter = 0;
    
    wire [31:0] corr_a_swap_counter_up = corr_a_swap_counter + (freq_swap);
    reg [31:0] corr_a_swap = 0;
    always @(posedge clk) begin
        corr_a_adc_channel <= adc_out_adc_channel;
        corr_a_dac_channel <= adc_out_dac_channel;
        corr_a_wave_i <= corr_wave_table_i[adc_out_phase[SEQ_PHASE_BITS-1:SEQ_PHASE_BITS-CORR_WAVE_INDEX_BITS]];
        corr_a_wave_q <= corr_wave_table_q[adc_out_phase[SEQ_PHASE_BITS-1:SEQ_PHASE_BITS-CORR_WAVE_INDEX_BITS]];
        corr_a_adc_value <= adc_out_adc_value;
        
        corr_a_mat_index <= adc_out_adc_channel * DAC_CHANNELS + (adc_out_dac_channel);
        corr_a_swap_counter <= corr_a_swap_counter_up;
        if (adc_out_adc_channel == 0 && adc_out_dac_channel == 0) begin
            corr_a_swap <= corr_a_swap + 1;
            if (corr_a_swap_counter >= FREQ_CLOCK) begin
                corr_a_swap_counter <= corr_a_swap_counter_up - FREQ_CLOCK;
                corr_a_swap <= 0;
            end
        end
    end

    reg [31:0] corr_b_swap = 0;
    reg [ADC_CHANNEL_INDEX_BITS-1:0] corr_b_adc_channel = 0;
    reg [DAC_CHANNEL_INDEX_BITS-1:0] corr_b_dac_channel = 0;
    reg signed [CORR_ACCU_BITS-1:0] corr_b_delta_i = 0;
    reg signed [CORR_ACCU_BITS-1:0] corr_b_delta_q = 0;
    reg [CORR_MAT_INDEX_BITS-1:0] corr_b_mat_index = 0;
    reg signed [CORR_ACCU_BITS-1:0] corr_b_accu_i = 0;
    reg signed [CORR_ACCU_BITS-1:0] corr_b_accu_q = 0;
    always @(posedge clk) begin
        corr_b_swap <= corr_a_swap;
        corr_b_adc_channel <= corr_a_adc_channel;
        corr_b_dac_channel <= corr_a_dac_channel;
        corr_b_delta_i <= corr_a_adc_value * corr_a_wave_i;
        corr_b_delta_q <= corr_a_adc_value * corr_a_wave_q;
        corr_b_mat_index <= corr_a_mat_index;
        corr_b_accu_i <= corr_accu_mat_i[corr_a_mat_index];
        corr_b_accu_q <= corr_accu_mat_q[corr_a_mat_index];
    end

    reg [CORR_MAT_INDEX_BITS-1:0] corr_c_mat_index = 0;
    reg signed [CORR_ACCU_BITS-1:0] corr_c_accu_i = 0;
    reg signed [CORR_ACCU_BITS-1:0] corr_c_accu_q = 0;
    always @(posedge clk) begin
        corr_c_mat_index <= corr_b_mat_index;
        out_valid <= 0;
        if (corr_b_swap == 0) begin
            out_valid <= 1;
            out_dac <= corr_b_dac_channel;
            out_adc <= corr_b_adc_channel;
            out_phase <= 0;
            out_data <= corr_b_accu_i;
            corr_c_accu_i <= corr_b_delta_i;
        end else begin
            corr_c_accu_i <= corr_b_accu_i + corr_b_delta_i;
        end
        if (corr_b_swap == 1) begin
            out_valid <= 1;
            out_dac <= corr_b_dac_channel;
            out_adc <= corr_b_adc_channel;
            out_phase <= 1;
            out_data <= corr_b_accu_q;
            corr_c_accu_q <= corr_b_delta_q;
        end else begin
            corr_c_accu_q <= corr_b_accu_q + corr_b_delta_q;
        end
    end
    
    always @(posedge clk) begin
        corr_accu_mat_i[corr_c_mat_index] <= corr_c_accu_i;
        corr_accu_mat_q[corr_c_mat_index] <= corr_c_accu_q;
    end

endmodule
