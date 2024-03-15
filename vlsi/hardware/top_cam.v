// 2021-2024 Philipp Ruppel

`include "memory.v"
`include "cpu.v"
`include "serial.v"
`include "clock.v"
`include "gpio.v"
`include "oram4.v"
`include "jtag.v"
`include "packet_rx_device.v"
`include "net_ecp5.v"
`include "thermometer.v"

module top #(
    parameter CAMERA_BIT_DEPTH = 12,
    parameter GPIO_CHANNELS = 8,
) (
    output wire uart_tx,
    inout wire [GPIO_CHANNELS-1:0] gpio_pins,
    output wire pin_camera_extclk,
    input wire pin_camera_pixclk,
    input wire [CAMERA_BIT_DEPTH-1:0] pin_camera_data,
    input wire pin_camera_frame_valid,
    input wire pin_camera_line_valid,
    input wire pin_mclk,
    inout wire [7:0] pin_ram_d,
    inout wire pin_ram_q,
    output wire pin_ram_ce,
    output wire pin_ram_clk,
    output wire pin_ram_reset,
    inout wire test_o_clk,
    inout wire test_o_dat
);
    localparam INVERT_CLOCK = 0;
    localparam INVERT_DATA = 0;
                                                                                                                                                                             
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
    (* global *)
    wire ram_clk;
    (* global *)
    wire ram_clk_shift;
    (* global *)
    wire clk;
 
    localparam CLKFREQ = 50*1000*1000;
    (* FREQUENCY_PIN_CLKI="40" *)
    (* FREQUENCY_PIN_CLKOP="150" *)
    (* FREQUENCY_PIN_CLKOS="150" *)
    (* FREQUENCY_PIN_CLKOS2="50" *)
    (* FREQUENCY_PIN_CLKOS3="100" *)
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
        .CLKOS_CPHASE(3),
        .CLKOS_FPHASE(0),
        .CLKOS2_ENABLE("ENABLED"),
        .CLKOS2_DIV(12),
        .CLKOS2_CPHASE(2),
        .CLKOS2_FPHASE(0),
        .CLKOS3_ENABLE("ENABLED"),
        .CLKOS3_DIV(6),
        .CLKOS3_CPHASE(2),
        .CLKOS3_FPHASE(0),
        .FEEDBK_PATH("CLKOP"),
        .CLKFB_DIV(15)
    ) second_pll_inst (
        .RST(1'b0),
        .STDBY(1'b0),
        .CLKI(pin_mclk),
        .CLKOP(ram_clk),
        .CLKFB(ram_clk),
        .CLKOS(ram_clk_shift),
        .CLKOS2(clk),
        .CLKOS3(pin_camera_extclk),
        .CLKINTFB(),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b1),
        .PHASESTEP(1'b1),
        .PHASELOADREG(1'b1),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b0),
    );
    
    localparam STACK_SIZE = 1024*4;
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
                                    assign uart_tx = 0;
    `make_mem_signals(clock);
    clock #(
        .CLKFREQ(CLKFREQ)
    ) clock_inst (
        `connect_mem_signals(clock)
    );

    `make_mem_signals(gpio);
    gpio #(
        .CHANNELS(GPIO_CHANNELS)
    ) gpio_inst (
        `connect_mem_signals(gpio),
        .pins(gpio_pins)
    );
    
    wire pll_pixclock = pin_camera_pixclk;
    
    wire packet_tx_pop;
    wire packet_tx_nempty;
    wire [63:0] packet_tx_data;
    wire packet_tx_end;
            
    `make_mem_signals(oram);
    oram #(
    ) oram_inst (
        `connect_mem_signals(oram),
        .pin_ram_d(pin_ram_d),
        .pin_ram_q(pin_ram_q),
        .pin_ram_ce(pin_ram_ce),
        .pin_ram_clk(pin_ram_clk),
        .pin_ram_reset(pin_ram_reset),
        .ram_clk(ram_clk),
        .ram_clk_shift(ram_clk_shift),
        .pin_camera_pixclk(pll_pixclock),
        .pin_camera_data(pin_camera_data),
        .pin_camera_frame_valid(pin_camera_frame_valid),
        .pin_camera_line_valid(pin_camera_line_valid),
        .packet_out_clk(clk),
        .packet_out_pop(packet_tx_pop),
        .packet_out_nempty(packet_tx_nempty),
        .packet_out_data(packet_tx_data),
        .packet_out_end(packet_tx_end)
    );
 
    wire packet_rx_pop;
    wire packet_rx_nempty;
    wire [63:0] packet_rx_data;
    wire packet_rx_end;

    wire [7:0] phase_shift;
    wire tx_fifo_in_full;
    assign packet_tx_pop = packet_tx_nempty && !tx_fifo_in_full;
    wire tx_fifo_out_pop;
    wire tx_fifo_out_nempty;
    wire tx_fifo_out_end;
    wire [63:0] tx_fifo_out_data;
    packet_fifo #(
        .WORD_SIZE(64),
        .FIFO_DEPTH(1024 * 4)
    ) tx_fifo_inst (
        .in_clk(clk),
        .in_shift(packet_tx_pop),
        .in_data(packet_tx_data),
        .in_full(tx_fifo_in_full),
        .in_end(packet_tx_end),
        .out_clk(clk),
        .out_pop(tx_fifo_out_pop),
        .out_data(tx_fifo_out_data),
        .out_nempty(tx_fifo_out_nempty),
        .out_end(tx_fifo_out_end)
    );
 
    net_ecp5 #(
        .PHASES(8),
        .RX_FIFO(16),
        .TX_FIFO(1024)
    ) inet (
        .net_clks(net_clks),
        .net_pins({test_o_clk,test_o_dat}),
        .net_tx_clk(net_clks[1]),
        .tx_clk(clk),
        .tx_pop(tx_fifo_out_pop),
        .tx_nempty(tx_fifo_out_nempty),
        .tx_data(tx_fifo_out_data),
        .tx_end(tx_fifo_out_end),
        .rx_clk(clk),
        .rx_pop(packet_rx_pop),
        .rx_nempty(packet_rx_nempty),
        .rx_data(packet_rx_data),
        .rx_end(packet_rx_end),
        .rx_phase_shift(phase_shift)
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
    
    `make_mem_signals(jtag);
    jtag jtag_inst (
        `connect_mem_signals(jtag)
    );
    
    `make_mem_signals(thermometer);
    thermometer #(
    ) thermometer_inst (
        `connect_mem_signals(thermometer)
    );

    always @(*) begin
        mem_rdata = 0;
        mem_ready = 0;
        mem_error = 1;
        `memory_map_device(CODE_BASE, code_ram);
        `memory_map_device(32'h30000000, stack_ram);
        `memory_map_device(32'h40000000, thermometer);
        `memory_map_device(32'h50000000, jtag);
        `memory_map_device(32'h60000000, clock);
        `memory_map_device(32'h70000000, packetrx);
        `memory_map_device(32'h10000000, gpio);
        `memory_map_device(32'hB0000000, oram);
    end

endmodule
