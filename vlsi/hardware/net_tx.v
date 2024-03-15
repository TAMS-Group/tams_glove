// 2023-2024 Philipp Ruppel

`ifndef INC_NET_TX_V
`define INC_NET_TX_V

module net_tx #(
    parameter BITS = 64,
    parameter LANES = 1,
    parameter SYNC = 64'h307A1AFD8FE3A9DA
) (
    (* global *)
    input wire clk,

    input wire in_valid,
    input wire [BITS-1:0] in_data,
    input wire in_end,
    output reg in_pull,

    output reg [LANES-1:0] out_data,
    output reg out_txen
);

    reg [LANES-1:0] obuf_data = 0;
    reg obuf_txen = 0;
    always @(posedge clk) begin
        out_data <= obuf_data;
        out_txen <= obuf_txen;
    end

    initial begin
        in_pull = 0;
        out_data = 0;
        out_txen = 0;
    end

    integer ilane;

    localparam STATE_IDLE = 0;
    localparam STATE_DATA = 1;
    localparam STATE_HOLD = 2;
    localparam STATE_DELAY = 3;
    localparam STATE_PREAMBLE = 4;
    reg [2:0] state = STATE_IDLE;

    reg [BITS/LANES+2-1:0] buffer [LANES-1:0];
    initial begin
        for (ilane = 0; ilane < LANES; ilane = ilane + 1) begin
            buffer[ilane] = 0;
        end
    end

    reg [$clog2(BITS/LANES+2)-1:0] count = 0;

    reg [3:0] nresync = 0;

    always @(posedge clk) begin
        in_pull <= 0;
        count <= count - 1;
        case (state)
            STATE_PREAMBLE: begin
                obuf_txen <= 1;
                obuf_data <= {LANES{count[0]}};
                if (count == 0) begin
                    state <= STATE_DATA;
                    for (ilane = 0; ilane < LANES; ilane = ilane + 1) begin
                        buffer[ilane] <= (SYNC >> (ilane * (BITS / LANES)));
                    end
                    count <= BITS / LANES - 1;
                    nresync <= -1;
                end
            end
            STATE_DATA: begin
                obuf_txen <= 1;
                for (ilane = 0; ilane < LANES; ilane = ilane + 1) begin
                    obuf_data[ilane] <= buffer[ilane][0];
                    buffer[ilane] <= (buffer[ilane] >> 1);
                end
                if (count == 0) begin
                    if (in_valid && nresync > 0) begin
                        in_pull <= 1;
                        for (ilane = 0; ilane < LANES; ilane = ilane + 1) begin
                            buffer[ilane][0] <= in_end;
                            buffer[ilane][1] <= ~in_end;
                            buffer[ilane][BITS/LANES+2-1:2] <= (in_data >> (ilane * (BITS / LANES)));
                        end
                        count <= BITS / LANES + 2 - 1;
                        nresync <= nresync - 1;
                    end else begin
                        state <= STATE_HOLD;
                        count <= 8;
                    end
                end
            end
            STATE_HOLD: begin
                obuf_data <= 0;
                obuf_txen <= 1;
                if (count == 0) begin
                    state <= STATE_DELAY;
                    count <= BITS / LANES - 1;
                end
            end
            STATE_DELAY: begin
                obuf_data <= 0;
                obuf_txen <= 0;
                if (count == 0) begin
                    state <= STATE_IDLE;
                end
            end
            default: begin
                obuf_txen <= 0;
                obuf_data <= 0;
                if (in_valid) begin
                    state <= STATE_PREAMBLE;
                    count <= BITS / LANES - 1;
                    obuf_txen <= 1;
                    obuf_data <= 0;
                end
            end
        endcase
    end

endmodule 

`endif
