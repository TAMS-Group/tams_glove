// 2023-2024 Philipp Ruppel

`include "net_tx.v"
`include "net_rx.v"

module top ( );

    localparam BITS = 64;
    localparam LANES = 2;
    localparam SYNC = 64'h307A1AFD8FE3A9DA;
    localparam PHASES = 6;

    localparam LANE_POLARITY = 4'b0101;

    initial begin
        $display("sync %64b", SYNC);
    end

    integer ok = 1;
    initial begin
        #2000000;
        ok = 0;
        #1;
        $display("");
    end

    reg tx_clk = 0;
    initial begin
        while (ok) begin
            tx_clk <= 1;
            #(PHASES*100);
            tx_clk <= 0;
            #(PHASES*100);
        end
    end

    localparam TEST_COUNTER_START = 123456789;

    integer tx_in_test_counter = TEST_COUNTER_START;

    reg tx_in_valid = 0;
    reg [BITS-1:0] tx_in_data = 0;
    reg tx_in_end = 0;
    wire tx_in_pull;
    always @(posedge tx_clk) begin
        if (!tx_in_valid && (($random % 5) == 0)) begin
        // if (!tx_in_valid && (($random % 50) == 0)) begin
            tx_in_valid <= 1;
            tx_in_data <= tx_in_test_counter;
            tx_in_end <= (tx_in_test_counter % 5 == 0);
            tx_in_test_counter <= tx_in_test_counter + 1;
        end
        if (tx_in_pull && tx_in_valid) begin
            tx_in_valid <= 0;
        end
    end

    wire [LANES-1:0] tx_out_data;
    wire tx_out_txen;

    net_tx #(
        .BITS(BITS),
        .LANES(LANES),
        .SYNC(SYNC)
    ) tx_inst (
        .clk(tx_clk),
        .in_valid(tx_in_valid),
        .in_data(tx_in_data),
        .in_end(tx_in_end),
        .in_pull(tx_in_pull),
        .out_data(tx_out_data),
        .out_txen(tx_out_txen)
    );

    reg [LANES-1:0] rx_in_data;
    real cables1 [LANES-1:0];
    real cables [LANES-1:0];
    integer icable;
    initial begin
        while (ok) begin
            for (icable = 0; icable < LANES; icable = icable + 1) begin
                cables[icable] = cables[icable] * 0.9 + (tx_out_txen ? tx_out_data[icable] : 0.5) * 0.1;
                cables[icable] = cables[icable] + (($random % 2) ? 1.0 : -1.0) * 0.03;
                rx_in_data[icable] = ((cables[icable] > 0.5) ^ LANE_POLARITY[icable]);
            end
            #50;
        end
    end

    reg [PHASES/2-1:0] rx_clks = 0;
    integer rx_clk_i = 0;
    initial begin
        while (ok) begin
            for (rx_clk_i = 0; rx_clk_i < PHASES/2; rx_clk_i = rx_clk_i + 1) begin
                rx_clks[rx_clk_i] <= 1;
                #200;
            end
            for (rx_clk_i = 0; rx_clk_i < PHASES/2; rx_clk_i = rx_clk_i + 1) begin
                rx_clks[rx_clk_i] <= 0;
                #200;
            end
        end
    end

    wire [BITS-1:0] rx_out_data;
    wire rx_out_valid;
    wire rx_out_end;

    integer out_i_ref = TEST_COUNTER_START;

    always @(posedge rx_clks[0]) begin
        if (rx_out_valid) begin
            $display("out %d %d expect %d", rx_out_end, rx_out_data, out_i_ref);
            out_i_ref <= out_i_ref + 1;
        end
    end

    wire [LANES*PHASES-1:0] rx_dbg_inputs;
    wire [LANES*PHASES-1:0] rx_dbg_samples;
    wire [PHASES-1:0] rx_dbg_sync;
    wire [$clog2(PHASES)-1:0] rx_dbg_phase;
    wire [LANES-1:0] rx_dbg_polarity;

    net_rx #(
        .PHASES(PHASES),
        .BITS(BITS),
        .LANES(LANES),
        .SYNC(SYNC)
    ) rx_inst(
        .clks(rx_clks),
        .in_data(rx_in_data),
        .out_data(rx_out_data),
        .out_end(rx_out_end),
        .out_valid(rx_out_valid),
        .dbg_inputs(rx_dbg_inputs),
        .dbg_samples(rx_dbg_samples),
        .dbg_sync(rx_dbg_sync),
        .dbg_phase(rx_dbg_phase),
        .dbg_polarity(rx_dbg_polarity),
        .enable(tx_out_txen),
        .in_phase_shift(2)
    );

    wire real cable0 = cables[0];
    wire real cable1 = cables[1];

    initial begin
        if (1) begin
            $dumpfile("net_tb.vcd");
            $dumpvars(0, tx_clk, tx_out_data, tx_out_txen, rx_clks, rx_dbg_inputs, rx_dbg_samples, rx_in_data);
            $dumpvars(0, cable0);
            $dumpvars(0, cable1);
            $dumpvars(0, rx_dbg_sync);
            $dumpvars(0, rx_dbg_phase);
            $dumpvars(0, rx_dbg_polarity);
        end
    end

endmodule