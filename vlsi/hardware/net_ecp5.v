// 2021-2024 Philipp Ruppel

`include "net_rx.v"
`include "net_tx.v"

`include "packet_fifo.v"

module net_ecp5 #(
    parameter DIFFRESISTOR = "100",
    parameter TX_FIFO = 256,
    parameter RX_FIFO = 256,
    parameter BITS = 64,
    parameter LANES = 2,
    parameter SYNC = 64'h307A1AFD8FE3A9DA,
    parameter PHASES = 6,
    parameter SCRAMBLING = 64'h18BD538CE5606E51,
) (
    input wire [PHASES/2-1:0] net_clks,
    inout wire [LANES-1:0] net_pins,

    input wire net_tx_clk,

    input wire [$clog2(PHASES)-1:0] rx_phase_shift,

    input wire tx_clk,
    output wire tx_pop,
    input wire tx_nempty,
    input wire [63:0] tx_data,
    input tx_end,

    input wire rx_clk,
    input wire rx_pop,
    output wire rx_nempty,
    output wire [63:0] rx_data,
    output rx_end,
);

    (* global *)
    wire net_clk = net_tx_clk;

    wire net_pins_t;

    wire [LANES-1:0] net_pins_o;
    wire [LANES-1:0] net_pins_i;

    (* IO_TYPE="LVCMOS25D", PULLMODE="NONE", HYSTERESIS="OFF", DIFFRESISTOR=DIFFRESISTOR, DRIVE=8, SLEWRATE="SLOW" *)
    TRELLIS_IO #( 
        .DIR("BIDIR") 
    ) io_test_i_clk [LANES-1:0] ( 
        .B(net_pins), 
        .I(net_pins_o), 
        .O(net_pins_i), 
        .T(net_pins_t) 
    );

    wire tx_fifo_in_full;
    assign tx_pop = tx_nempty && !tx_fifo_in_full;
    wire tx_fifo_out_pop;
    wire tx_fifo_out_nempty;
    wire tx_fifo_out_end;
    wire [BITS-1:0] tx_fifo_out_data;
    packet_fifo #(
        .WORD_SIZE(BITS),
        .FIFO_DEPTH(TX_FIFO)
    ) tx_fifo_inst (
        .in_clk(tx_clk),
        .in_shift(tx_pop),
        .in_data(tx_data),
        .in_full(tx_fifo_in_full),
        .in_end(tx_end),
        .out_clk(net_clk),
        .out_pop(tx_fifo_out_pop),
        .out_data(tx_fifo_out_data),
        .out_nempty(tx_fifo_out_nempty),
        .out_end(tx_fifo_out_end)
    );

    wire net_txen;

    net_tx #(
        .BITS(BITS),
        .LANES(LANES),
        .SYNC(SYNC)
    ) tx_inst (
        .clk(net_clk),
        .in_valid(tx_fifo_out_nempty),
        .in_data(tx_fifo_out_data),
        .in_end(tx_fifo_out_end),
        .in_pull(tx_fifo_out_pop),
        .out_data(net_pins_o),
        .out_txen(net_txen)
    );

    assign net_pins_t = !net_txen;

    wire dec_out_valid;
    wire dec_out_end;
    wire [BITS-1:0] dec_out_data;

    net_rx #(
        .PHASES(PHASES),
        .BITS(BITS),
        .LANES(LANES),
        .SYNC(SYNC)
    ) rx_inst(
        .clks(net_clks),
        .in_data(net_pins_i),
        .enable(net_pins_t),
        .out_data(dec_out_data),
        .out_end(dec_out_end),
        .out_valid(dec_out_valid),
        .in_phase_shift(rx_phase_shift),
    );

    wire rx_fifo_in_full;
    wire rx_fifo_in_shift = (!rx_fifo_in_full && dec_out_valid);
    wire [63:0] rx_fifo_out_data;
    packet_fifo #(
        .WORD_SIZE(BITS),
        .FIFO_DEPTH(RX_FIFO)
    ) rx_fifo_inst (
        .in_clk(net_clks[0]),
        .in_shift(rx_fifo_in_shift),
        .in_data(dec_out_data),
        .in_full(rx_fifo_in_full),
        .in_end(dec_out_end),
        .out_clk(rx_clk),
        .out_pop(rx_pop),
        .out_data(rx_fifo_out_data),
        .out_nempty(rx_nempty),
        .out_end(rx_end)
    );
    assign rx_data = rx_fifo_out_data;

endmodule