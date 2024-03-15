// 2021-2024 Philipp Ruppel

`include "utils.v"

module usb_fifo (

    input wire in_nempty,
    output wire in_pop,
    input wire [31:0] in_data,
    
	output wire out_shift,
	output wire [31:0] out_data,
	input wire out_full,

    input wire pin_clk,
    input wire pin_txen,
    input wire pin_rxfn,
    output wire pin_wrn,
    output wire pin_rdn,
    output wire pin_oen,
    inout wire [3:0] pin_be,
    inout wire [31:0] pin_data

);

    wire [31:0] data_in;
    wire data_oe;
    wire [31:0] data_out;
    (* IO_TYPE="LVCMOS33", PULLMODE="UP", DRIVE="4" *)
    TRELLIS_IO #( .DIR("BIDIR") ) io_data_inst [31:0] ( .B(pin_data), .I(data_out), .O(data_in), .T(~data_oe) );

    wire [3:0] be_in;
    wire be_oe;
    wire [3:0] be_out;
    (* IO_TYPE="LVCMOS33", PULLMODE="UP", DRIVE="4" *)
    TRELLIS_IO #( .DIR("BIDIR") ) io_be_inst [3:0] ( .B(pin_be), .I(be_out), .O(be_in), .T(~be_oe) );

    localparam STATE_IDLE = 0;
    localparam STATE_UPSTREAM = 1;
    localparam STATE_DOWNSTREAM_BEGIN = 2;
    localparam STATE_DOWNSTREAM = 3;
    localparam STATE_DOWNSTREAM_END = 4;

    reg [2:0] state = STATE_IDLE;

    always @(posedge pin_clk) begin
        case (state)
            STATE_IDLE: begin
                if (!pin_rxfn) begin
                    state <= STATE_DOWNSTREAM_BEGIN;
                end else if (!pin_txen && in_nempty) begin
                    state <= STATE_UPSTREAM;
                end
            end
            STATE_UPSTREAM: begin
                if (pin_txen || !in_nempty) begin
                    state <= STATE_IDLE;
                end
            end
            STATE_DOWNSTREAM_BEGIN: begin
                state <= STATE_DOWNSTREAM;
            end
            STATE_DOWNSTREAM: begin
                if (pin_rxfn) begin
                    state <= STATE_DOWNSTREAM_END;
                end
            end
            STATE_DOWNSTREAM_END: begin
                state <= STATE_IDLE;
            end
        endcase
    end

    assign out_data = data_in;

    assign pin_oen = !((state == STATE_DOWNSTREAM_BEGIN) || (state == STATE_DOWNSTREAM));
    assign in_pop = (in_nempty && !pin_txen && state == STATE_UPSTREAM);
    assign out_shift = (!out_full && !pin_rxfn && state == STATE_DOWNSTREAM);

    assign be_oe = in_pop;
    assign be_out = 4'b1111;

    assign data_oe = in_pop;
    assign data_out = in_data;

    assign pin_wrn = !in_pop;
    assign pin_rdn = !out_shift;

endmodule
