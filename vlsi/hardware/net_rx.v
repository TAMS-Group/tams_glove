// 2023-2024 Philipp Ruppel

`ifndef INC_NET_RX_V
`define INC_NET_RX_V

module net_rx #(
    parameter PHASES = 4,
    parameter BITS = 64,
    parameter LANES = 1,
    parameter SYNC = 64'h307A1AFD8FE3A9DA
) (
    (* global *)
    input wire [PHASES/2-1:0] clks,

    input wire enable,

    input wire [LANES-1:0] in_data,

    input wire [$clog2(PHASES)-1:0] in_phase_shift,

    output reg [BITS-1:0] out_data,
    output reg out_end,
    output reg out_valid,

    output reg [LANES*PHASES-1:0] dbg_inputs,
    output wire [LANES*PHASES-1:0] dbg_samples,
    output wire [PHASES-1:0] dbg_sync,
    output wire [$clog2(PHASES)-1:0] dbg_phase,
    output wire [LANES-1:0] dbg_polarity
);

    integer ilane;
    integer iphase;

    genvar gbit;
    genvar glane;
    genvar gphase;

    (* global *)
    wire clk = clks[0]; 

    reg [LANES*PHASES-1:0] reg_samples = 0;
    assign dbg_samples = reg_samples;
    generate
        for (gphase = 0; gphase < PHASES/2; gphase = gphase + 1) begin

            reg [LANES-1:0] in_pos = 0;
            always @(posedge clks[gphase]) begin
                in_pos <= in_data;
            end
            reg [LANES-1:0] sample_pos_0 = 0;
            always @(posedge clks[gphase]) begin
                sample_pos_0 <= in_pos;
                dbg_inputs[LANES*(gphase+1)-1:LANES*(gphase+0)] <= in_pos;
            end
            reg [LANES-1:0] sample_pos_1 = 0;
            always @(posedge clks[0]) begin
                sample_pos_1 <= sample_pos_0;
            end
            always @(posedge clks[0]) begin
                reg_samples[LANES*(gphase+1)-1:LANES*(gphase+0)] <= sample_pos_1;
            end

            reg [LANES-1:0] in_neg = 0;
            always @(negedge clks[gphase]) begin
                in_neg <= in_data;
            end
            reg [LANES-1:0] sample_neg_0 = 0;
            always @(negedge clks[gphase]) begin
                sample_neg_0 <= in_neg;
                dbg_inputs[LANES*(PHASES/2+gphase+1)-1:LANES*(PHASES/2+gphase+0)] <= in_neg;
            end
            reg [LANES-1:0] sample_neg_1 = 0;
            always @(negedge clks[0]) begin
                sample_neg_1 <= sample_neg_0;
            end
            always @(posedge clks[0]) begin
                reg_samples[LANES*(PHASES/2+gphase+1)-1:LANES*(PHASES/2+gphase+0)] <= sample_neg_1;
            end

        end
    endgenerate

    reg [LANES*PHASES-1:0] samples = 0;
    reg reg_enable = 0;
    always @(posedge clk) begin
        reg_enable <= enable;
        if (reg_enable)
            samples <= reg_samples;
        else
            samples <= 0;
    end

    localparam LANE_BITS = BITS / LANES;

    reg [LANES-1:0] sync_lane_phase_invert [PHASES-1:0];

    reg [PHASES-1:0] sync_match;

    assign dbg_sync = sync_match;
    reg [LANE_BITS-1:0] plbuffers [PHASES*LANES-1:0];

    always @(posedge clk) begin
        for (iphase = 0; iphase < PHASES; iphase = iphase + 1) begin
            sync_lane_phase_invert[iphase] <= 0;
            sync_match[iphase] <= 1;
            for (ilane = 0; ilane < LANES; ilane = ilane + 1) begin
                plbuffers[LANES*iphase+ilane] <= {samples[LANES*iphase+ilane], plbuffers[LANES*iphase+ilane][BITS/LANES-1:1]};
                sync_lane_phase_invert[iphase][ilane] <= 0;
                if (plbuffers[LANES*iphase+ilane] == ((SYNC >> (ilane * (BITS / LANES))) & {LANE_BITS{1'b1}})) begin
                    sync_lane_phase_invert[iphase][ilane] <= 0;
                end else if (plbuffers[LANES*iphase+ilane] == ((~SYNC >> (ilane * (BITS / LANES))) & {LANE_BITS{1'b1}})) begin
                    sync_lane_phase_invert[iphase][ilane] <= 1;
                end else begin
                    sync_match[iphase] <= 0;
                end
            end
        end
    end

    reg scan_wait = 0;
    reg scan_lock = 0;
    reg [$clog2(PHASES)-1:0] scan_phase = 0;
    reg [LANES-1:0] scan_invert = 0;
    assign dbg_polarity = scan_invert;
    always @(posedge clk) begin
        scan_phase <= 0;
        scan_wait <= 0;
        scan_invert <= 0;
        scan_lock <= 0;
        for (iphase = PHASES - 1; iphase >= 0; iphase = iphase - 1) begin
            if (sync_match[iphase]) begin
                scan_phase <= ((iphase + in_phase_shift) % PHASES);
                scan_wait <= ((iphase + in_phase_shift) >= PHASES);
                scan_invert <= sync_lane_phase_invert[iphase];
                scan_lock <= 1;
            end
        end
    end

    reg [$clog2(PHASES)-1:0] dec_phase = 0;
    assign dbg_phase = dec_phase;
    reg [BITS/LANES-1:0] dec_buffers [LANES-1:0];
    reg [LANES-1:0] dec_invert = 0;
    always @(posedge clk) begin
        for (ilane = 0; ilane < LANES; ilane = ilane + 1) begin
            if (dec_invert[ilane]) 
                dec_buffers[ilane] <= ~plbuffers[LANES*dec_phase+ilane];
            else
                dec_buffers[ilane] <= plbuffers[LANES*dec_phase+ilane];
        end
    end

    wire [BITS-1:0] flat_buffer;
    generate
        for (glane = 0; glane < LANES; glane = glane + 1) begin
            assign flat_buffer[(glane+1)*(BITS/LANES)-1:glane*(BITS/LANES)] = dec_buffers[glane];
        end
    endgenerate

    wire [LANES*2-1:0] dec_buffer_command;
    generate
        for (glane = 0; glane < LANES; glane = glane + 1) begin
            assign dec_buffer_command[glane] = dec_buffers[glane][BITS/LANES-2];
            assign dec_buffer_command[LANES+glane] = dec_buffers[glane][BITS/LANES-3];
        end
    endgenerate

    localparam STATE_IDLE = 0;
    localparam STATE_COMMAND = 1;
    localparam STATE_DATA = 2;
    reg [1:0] state = STATE_IDLE;
    reg [$clog2(BITS/LANES+2)-1:0] delay = 0;

    always @(posedge clk) begin
        out_data <= 0;
        out_valid <= 0;
        delay <= delay - 1;
        case (state)
            STATE_COMMAND: begin
                if (delay == 0) begin
                    state <= STATE_DATA;
                    case (dec_buffer_command) 
                        {{LANES{1'b0}},{LANES{1'b1}}}: begin
                            delay <= BITS / LANES - 2;
                            out_end <= 0;
                        end
                        {{LANES{1'b1}},{LANES{1'b0}}}: begin
                            delay <= BITS / LANES - 2;
                            out_end <= 1;
                        end
                        default: begin
                            state <= STATE_IDLE;
                        end
                    endcase
                end
            end
            STATE_DATA: begin
                if (delay == 0) begin
                    state <= STATE_COMMAND;
                    delay <= 2;
                    out_data <= flat_buffer;
                    out_valid <= 1;
                end
            end
            default: begin
                if (scan_lock) begin
                    dec_phase <= scan_phase;
                    state <= STATE_COMMAND;
                    delay <= scan_wait + 1;
                    dec_invert <= scan_invert;
                end
            end
        endcase
    end

endmodule

`endif