// 2021-2024 Philipp Ruppel

`define NET_NOGLOBAL

`include "led_controller.v"
`include "memory.v"
`include "cpu.v"
`include "clock.v"
`include "gpio.v"
`include "jtag.v"
`include "packet_device.v"
`include "packet_mux.v"
`include "packet_fifo.v"
`include "usb_fifo.v"
`include "packet_framer.v"
`include "ringbuffer.v"
`include "usb_rx.v"
`include "event_counter.v"
`include "net_ecp5.v"

module top #(
    parameter NETWORK_PORTS=8
) (
    input wire mclk,
    input wire pin_clk,
    input wire pin_txen,
    input wire pin_rxfn,
    output wire pin_wrn,
    output wire pin_rdn,
    output wire pin_oen,
    inout wire [3:0] pin_be,
    inout wire [31:0] pin_data,
    inout wire [NETWORK_PORTS-1:0] pin_net_clk,
    inout wire [NETWORK_PORTS-1:0] pin_net_dat,
    output wire pin_ledctrl
);

    localparam STACK_SIZE = 1024*1;
    localparam CODE_SIZE = 1024*4;
    localparam CODE_BASE = 32'h0;


    (* global *)
    wire clk;

    (* global *)
    wire [3:0] net_clks;

    localparam CLKFREQ = 40*1000*1000;
    (* FREQUENCY_PIN_CLKI="40" *)
    (* FREQUENCY_PIN_CLKOP="150" *)
    (* FREQUENCY_PIN_CLKOS="150" *)
    (* FREQUENCY_PIN_CLKOS2="150" *)
    (* FREQUENCY_PIN_CLKOS3="150" *)
    (* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
    EHXPLLL #(
        .PLLRST_ENA("DISABLED"),
        .INTFB_WAKE("DISABLED"),
        .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"),
        .OUTDIVIDER_MUXA("DIVA"),
        .OUTDIVIDER_MUXB("DIVB"),
        .OUTDIVIDER_MUXC("DIVC"),
        .OUTDIVIDER_MUXD("DIVD"),
        .CLKI_DIV(4),
        .CLKOP_ENABLE("ENABLED"),
        .CLKOP_DIV(4),
        .CLKOP_CPHASE(2),
        .CLKOP_FPHASE(0),
        .CLKOS_ENABLE("ENABLED"),
        .CLKOS_DIV(4),
        .CLKOS_CPHASE(2),
        .CLKOS_FPHASE(4),
        .CLKOS2_ENABLE("ENABLED"),
        .CLKOS2_DIV(4),
        .CLKOS2_CPHASE(3),
        .CLKOS2_FPHASE(0),
        .CLKOS3_ENABLE("ENABLED"),
        .CLKOS3_DIV(4),
        .CLKOS3_CPHASE(3),
        .CLKOS3_FPHASE(4),
        .FEEDBK_PATH("CLKOP"),
        .CLKFB_DIV(15)
    ) pll_m (
        .RST(1'b0),
        .STDBY(1'b0),
        .CLKI(mclk),
        .CLKOP(net_clks[0]),
        .CLKFB(net_clks[0]),
        .CLKINTFB(),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b1),
        .PHASESTEP(1'b1),
        .PHASELOADREG(1'b1),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b0),
        .CLKOS(net_clks[1]),
        .CLKOS2(net_clks[2]),
        .CLKOS3(net_clks[3])
    );

    assign clk = mclk;

    (* global *)
    wire packet_clk;
 
 
    wire mem_valid;
    wire [31:0] mem_addr;
    wire [3:0] mem_wstrb;
    wire [31:0] mem_wdata;
    wire [31:0] mem_rdata;
    wire mem_ready;
    wire mem_error;
    cpu #(
        .STACK_START(32'h30000000 + STACK_SIZE),
        .PROGRAM_START(32'h10000)
    ) cpu_inst (
        .clk(clk),
        .mem_valid(mem_valid),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),
        .mem_addr(mem_addr),
        .mem_ready(mem_ready),
        .mem_error(mem_error)
    );


    wire [31:0] dev_mem_addr;
    wire [31:0] mem_device;
    always @(*) begin
        dev_mem_addr[27:0] = mem_addr[27:0];
        dev_mem_addr[31:28] = 0;
        mem_device[27:0] = 28'h0000000;
        mem_device[31:28] = mem_addr[31:28];
    end


    `make_mem_signals(code_ram);
    memory #(
        .MEM_WORDS(CODE_SIZE),
        .FILE_NAME("../build/random.mem")
    ) code_ram_inst (
        `connect_mem_signals(code_ram)
    );


    `make_mem_signals(stack_ram);
    memory #(
        .MEM_WORDS(STACK_SIZE)
    ) stack_ram_inst (
        `connect_mem_signals(stack_ram)
    );


    `make_mem_signals(jtag);
    jtag jtag_inst (
        `connect_mem_signals(jtag)
    );


    `make_mem_signals(clock);
    clock #(
        .CLKFREQ(CLKFREQ)
    ) clock_inst (
        `connect_mem_signals(clock)
    );



    wire usb_clk_fb;
    wire usb_clk;
    (* FREQUENCY_PIN_CLKI="100" *)
    (* FREQUENCY_PIN_CLKOP="100" *)
    (* FREQUENCY_PIN_CLKOS="100" *)
    (* FREQUENCY_PIN_CLKOS2="75" *)
    (* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
    EHXPLLL #(
        .PLLRST_ENA("DISABLED"),
        .INTFB_WAKE("DISABLED"),
        .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"),
        .OUTDIVIDER_MUXA("DIVA"),
        .OUTDIVIDER_MUXB("DIVB"),
        .OUTDIVIDER_MUXC("DIVC"),
        .OUTDIVIDER_MUXD("DIVD"),
        .CLKI_DIV(1),
        .CLKOP_ENABLE("ENABLED"),
        .CLKOP_DIV(6),
        .CLKOP_CPHASE(2),
        .CLKOP_FPHASE(0),
        .CLKOS_ENABLE("ENABLED"),
        .CLKOS_DIV(6),
        .CLKOS_CPHASE(6),
        .CLKOS_FPHASE(4),
        .CLKOS2_ENABLE("ENABLED"),
        .CLKOS2_DIV(8),
        .CLKOS2_CPHASE(2),
        .CLKOS2_FPHASE(0),
        .FEEDBK_PATH("CLKOP"),
        .CLKFB_DIV(1)
    ) pll_i (
        .RST(1'b0),
        .STDBY(1'b0),
        .CLKI(pin_clk),
        .CLKOP(usb_clk_fb),
        .CLKOS(usb_clk),
        .CLKOS2(packet_clk),
        .CLKFB(usb_clk_fb),
        .CLKINTFB(),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b1),
        .PHASESTEP(1'b1),
        .PHASELOADREG(1'b1),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b0)
    );


    localparam MUX_WORD_SIZE = 32;
    localparam MUX_INPUT_WORDS = 2;
    localparam MUX_CHANNEL_COUNT = NETWORK_PORTS;

    wire [MUX_CHANNEL_COUNT-1:0] mux_in_full;
    wire [MUX_CHANNEL_COUNT-1:0] mux_in_shift;
    wire [MUX_CHANNEL_COUNT-1:0] mux_in_end;
    wire [MUX_CHANNEL_COUNT*MUX_INPUT_WORDS*MUX_WORD_SIZE-1:0] mux_in_data;


    localparam PACKET_DEVICE_CHANNELS = NETWORK_PORTS;
    `make_mem_signals(packet_device);
    wire [PACKET_DEVICE_CHANNELS-1:0] packet_device_out_pop;
    wire [PACKET_DEVICE_CHANNELS-1:0] packet_device_out_nempty;
    wire [63:0] packet_device_out_data;
    wire packet_device_out_end;
    wire [7:0] phase_shift;
    packet_device #(
        .CHANNELS(PACKET_DEVICE_CHANNELS)
    ) packet_device_inst (
        `connect_mem_signals(packet_device),
        .out_pop(packet_device_out_pop),
        .out_nempty(packet_device_out_nempty),
        .out_data(packet_device_out_data),
        .out_end(packet_device_out_end),
        .out_phase_shift(phase_shift),
    );

    genvar g;

    generate

        for (g = 0; g < NETWORK_PORTS; g = g + 1) begin

            wire packet_des_out_pop;
            wire packet_des_out_nempty;
            wire [63:0] packet_des_out_data;
            wire packet_des_out_end;

            wire [$clog2(8)-1:0] dbg_phase;

            net_ecp5 #(
                .DIFFRESISTOR("OFF"),
                .RX_FIFO(8),
                .TX_FIFO(8),
                .PHASES(8)
            ) inet (
                .net_clks(net_clks),
                .net_pins({pin_net_clk[g],pin_net_dat[g]}),
                .tx_clk(clk),
                .tx_pop(packet_device_out_pop[g]),
                .tx_nempty(packet_device_out_nempty[g]),
                .tx_data(packet_device_out_data),
                .tx_end(packet_device_out_end),
                .net_tx_clk(net_clks[g % 4]),
                .rx_clk(packet_clk),
                .rx_pop(packet_des_out_pop),
                .rx_nempty(packet_des_out_nempty),
                .rx_data(packet_des_out_data),
                .rx_end(packet_des_out_end),
                .rx_phase_shift(phase_shift),
            );


            assign packet_des_out_pop = (!mux_in_full[g] && packet_des_out_nempty);
            assign mux_in_shift[g] = packet_des_out_pop;
            assign mux_in_data[MUX_INPUT_WORDS*MUX_WORD_SIZE*(g+1)-1:MUX_INPUT_WORDS*MUX_WORD_SIZE*g] = packet_des_out_data;
            assign mux_in_end[g] = packet_des_out_end;

        end

    endgenerate

    localparam MUX_OUTPUT_WORDS = 1;

    wire mux_out_pop;
    wire mux_out_nempty;
    wire [MUX_OUTPUT_WORDS*MUX_WORD_SIZE-1:0] mux_out_data;

    packet_mux #(
        .CHANNEL_COUNT(MUX_CHANNEL_COUNT),
        .WORD_SIZE(MUX_WORD_SIZE),
        .INPUT_WORDS(MUX_INPUT_WORDS),
        .OUTPUT_WORDS(MUX_OUTPUT_WORDS),
        .SEGMENT_SIZE(4),
        .HEADER_TEMPLATE(32'h23010000),
        .HEADER_COUNT_SHIFT(0),
        .HEADER_CHANNEL_SHIFT(8),
        .HEADER_END_SHIFT(12),
        .BUFFER_WORDS(24)
    ) mux_inst (
        .clk(packet_clk),

        .in_full(mux_in_full),
        .in_shift(mux_in_shift),
        .in_end(mux_in_end),
        .in_data(mux_in_data),

        .out_pop(mux_out_pop),
        .out_nempty(mux_out_nempty),
        .out_data(mux_out_data)
    );


    wire host_fifo_in_full;
    assign mux_out_pop = (!host_fifo_in_full && mux_out_nempty);
    wire host_fifo_out_pop;
    wire host_fifo_out_nempty;
    wire [31:0] host_fifo_out_data;



    async_fifo #(
        .WIDTH(32),
        .DEPTH(1024 * 16)
    ) host_fifo_inst (
        .in_clk(packet_clk),
        .in_shift(mux_out_pop),
        .in_data(mux_out_data),
        .in_full(host_fifo_in_full),
        .out_clk(usb_clk),
        .out_pop(host_fifo_out_pop),
        .out_data(host_fifo_out_data),
        .out_nempty(host_fifo_out_nempty)
    );



    wire usb_rx_fifo_in_shift;
    wire usb_rx_fifo_in_full;
    wire [31:0] usb_rx_fifo_in_data;

    usb_fifo usb_fifo_inst (
        .in_pop(host_fifo_out_pop),
        .in_nempty(host_fifo_out_nempty),
        .in_data(host_fifo_out_data),
        .out_shift(usb_rx_fifo_in_shift),
        .out_data(usb_rx_fifo_in_data),
        .out_full(usb_rx_fifo_in_full),
        .pin_clk(usb_clk),
        .pin_txen(pin_txen),
        .pin_rxfn(pin_rxfn),
        .pin_wrn(pin_wrn),
        .pin_rdn(pin_rdn),
        .pin_oen(pin_oen),
        .pin_be(pin_be),
        .pin_data(pin_data)
    );

    wire usb_rx_fifo_out_pop;
    wire usb_rx_fifo_out_nempty;
    wire [31:0] usb_rx_fifo_out_data;
    async_fifo #(
        .WIDTH(32),
        .DEPTH(512)
    ) usb_rx_fifo (
        .in_clk(usb_clk),
        .in_shift(usb_rx_fifo_in_shift),
        .in_full(usb_rx_fifo_in_full),
        .in_data(usb_rx_fifo_in_data),
        .out_clk(clk),
        .out_pop(usb_rx_fifo_out_pop),
        .out_nempty(usb_rx_fifo_out_nempty),
        .out_data(usb_rx_fifo_out_data)
    );
 
    `make_mem_signals(usbrx);
    usb_rx usbrx (
        `connect_mem_signals(usbrx),
        .in_pop(usb_rx_fifo_out_pop),
        .in_nempty(usb_rx_fifo_out_nempty),
        .in_data(usb_rx_fifo_out_data)
    );


    `make_mem_signals(led);
    led_controller #(
        .LEDCOUNT(9),
        .CLKFREQ(CLKFREQ)
    ) led_controller_inst (
        `connect_mem_signals(led),
        .led_pin(pin_ledctrl),
    );

    always @(*) begin
        mem_rdata = 0;
        mem_ready = 0;
        mem_error = 1;
        `memory_map_device(CODE_BASE, code_ram);
        `memory_map_device(32'h20000000, led);
        `memory_map_device(32'h30000000, stack_ram);
        `memory_map_device(32'h50000000, jtag);
        `memory_map_device(32'h60000000, clock);
        `memory_map_device(32'h70000000, usbrx);
        `memory_map_device(32'h90000000, packet_device);
    end

endmodule
