// 2021-2024 Philipp Ruppel

`include "memory.v"
`include "cpu.v"
`include "jtag.v"
`include "clock.v"
`include "led_controller.v"
`include "packet_device.v"
`include "packet_rx_device.v"
`include "gpio.v"
`include "net_ecp5.v"

module top #(
    parameter GPIO_CHANNELS = 11,
) (
    input wire pin_mclk,
    output wire pin_ledctrl,
    inout wire net_clk,
    inout wire net_dat,
    inout wire [GPIO_CHANNELS-1:0] gpio_pins
);

    (* global *)
    wire clk;

    (* global *)
    wire [3:0] net_clks;

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
    ) main_pll_inst (
        .RST(1'b0),
        .STDBY(1'b0),
        .CLKI(pin_mclk),
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
        .CLKOS2(net_clks[3])
    );

    localparam CLKFREQ = 50*1000*1000;
    (* FREQUENCY_PIN_CLKI="40" *)
    (* FREQUENCY_PIN_CLKOP="50" *)
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
        .CLKOP_DIV(12),
        .CLKOP_CPHASE(5),
        .CLKOP_FPHASE(0),
        .FEEDBK_PATH("CLKOP"),
        .CLKFB_DIV(5)
    ) second_pll_inst (
        .RST(1'b0),
        .STDBY(1'b0),
        .CLKI(pin_mclk),
        .CLKOP(clk),
        .CLKFB(clk),
        .CLKINTFB(),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b1),
        .PHASESTEP(1'b1),
        .PHASELOADREG(1'b1),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b0),
    );

    localparam STACK_SIZE = 1024*8;
    localparam CODE_SIZE = 1024*4;
    localparam CODE_BASE = 32'h0;

    wire mem_valid;
    wire [31:0] mem_addr;
    wire [3:0] mem_wstrb;
    wire [31:0] mem_wdata;
    wire [31:0] mem_rdata;
    wire mem_ready;
    wire mem_error;
    cpu #(
        .STACK_START(32'h30000000 + STACK_SIZE),
        .PROGRAM_START(CODE_BASE)
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

    `make_mem_signals(clock);
    clock #(
        .CLKFREQ(CLKFREQ)
    ) clock_inst (
        `connect_mem_signals(clock)
    );

    `make_mem_signals(jtag);
    jtag jtag_inst (
        `connect_mem_signals(jtag)
    );

    `make_mem_signals(led);
    led_controller #(
        .LEDCOUNT(1),
        .CLKFREQ(CLKFREQ)
    ) led_controller_inst (
        `connect_mem_signals(led),
        .led_pin(pin_ledctrl),
    );

    wire [7:0] phase_shift;

    wire packet_tx_pop;
    wire packet_tx_nempty;
    wire [63:0] packet_tx_data;
    wire packet_tx_end;

    wire packet_rx_pop;
    wire packet_rx_nempty;
    wire [63:0] packet_rx_data;
    wire packet_rx_end;

    net_ecp5 #(
        .PHASES(8),
        .RX_FIFO(16),
        .TX_FIFO(1024)
    ) inet (
        .net_clks(net_clks),
        .net_pins({net_clk,net_dat}),
        .net_tx_clk(net_clks[1]),
        .tx_clk(clk),
        .tx_pop(packet_tx_pop),
        .tx_nempty(packet_tx_nempty),
        .tx_data(packet_tx_data),
        .tx_end(packet_tx_end),
        .rx_clk(clk),
        .rx_pop(packet_rx_pop),
        .rx_nempty(packet_rx_nempty),
        .rx_data(packet_rx_data),
        .rx_end(packet_rx_end),
        .rx_phase_shift(phase_shift)
    );

    `make_mem_signals(packet_device);
    packet_device #(
        .CHANNELS(1)
    ) packet_device_inst (
        `connect_mem_signals(packet_device),
        .out_pop(packet_tx_pop),
        .out_nempty(packet_tx_nempty),
        .out_data(packet_tx_data),
        .out_end(packet_tx_end),
    );

    `make_mem_signals(packetrx);
    packet_rx_device packetrxdev (
        `connect_mem_signals(packetrx),
        .in_pop(packet_rx_pop),
        .in_nempty(packet_rx_nempty),
        .in_data(packet_rx_data),
        .in_end(packet_rx_end),
        .out_phase_shift(phase_shift)
    );

    `make_mem_signals(gpio);
    gpio #(
        .CHANNELS(GPIO_CHANNELS)
    ) gpio_inst (
        `connect_mem_signals(gpio),
        .pins(gpio_pins)
    );
    
    always @(*) begin
        mem_rdata = 0;
        mem_ready = 0;
        mem_error = 1;
        `memory_map_device(CODE_BASE, code_ram);
        `memory_map_device(32'h10000000, gpio);
        `memory_map_device(32'h20000000, led);
        `memory_map_device(32'h30000000, stack_ram);
        `memory_map_device(32'h50000000, jtag);
        `memory_map_device(32'h60000000, clock);
        `memory_map_device(32'h70000000, packetrx);
        `memory_map_device(32'h90000000, packet_device);
    end

endmodule
