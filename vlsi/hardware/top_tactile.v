// 2021-2024 Philipp Ruppel

`include "led_controller.v"
`include "memory.v"
`include "cpu.v"
`include "tac_new.v"
`include "tactile_device.v"
`include "clock.v"
`include "gpio_core.v"
`include "jtag.v"
`include "thermometer.v"
 
module top #(
  parameter GPIO_CHANNELS = 7,
) (
  output reg [15:0] tactx,
  input wire [15:0] tacrxi,
  output wire [15:0] tacrxo,
  input wire mclk,
  inout wire [GPIO_CHANNELS-1:0] gpio_pins,
  output wire wsled,
);

  localparam CLKFREQ = 50*1000*1000;
  wire clk;
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
  ) pll_i (
    .RST(1'b0),
    .STDBY(1'b0),
    .CLKI(mclk),
    .CLKOP(clk),
    .CLKFB(clk),
    .CLKINTFB(),
    .PHASESEL0(1'b0),
    .PHASESEL1(1'b0),
    .PHASEDIR(1'b1),
    .PHASESTEP(1'b1),
    .PHASELOADREG(1'b1),
    .PLLWAKESYNC(1'b0),
    .ENCLKOP(1'b0)
	);

  localparam STACK_SIZE = 1024*12;
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

  `make_mem_signals(tactile);
  tactile_device #(
    .FREQ_CLOCK(CLKFREQ),
    .ADC_CHANNELS(16),
    .DAC_CHANNELS(16)
  ) tactile_device_inst (
    `connect_mem_signals(tactile),
    .pins_adc_in(tacrxi),
    .pins_adc_out(tacrxo),
    .pins_dac_out(tactx)
  );

  `make_mem_signals(clock);
  clock #(
    .CLKFREQ(CLKFREQ)
  ) clock_inst (
    `connect_mem_signals(clock)
  );

  wire [GPIO_CHANNELS-1:0] gpio_i;
  wire [GPIO_CHANNELS-1:0] gpio_o;
  wire [GPIO_CHANNELS-1:0] gpio_t;

  `make_mem_signals(gpio);
  gpio_core #(
    .CHANNELS(GPIO_CHANNELS)
  ) gpio_core_inst (
    `connect_mem_signals(gpio),
    .pins_i(gpio_i),
    .pins_o(gpio_o),
    .pins_t(gpio_t)
  );

  genvar g;
  generate
    for (g = 0; g < GPIO_CHANNELS; g = g + 1) begin
      (* IO_TYPE="LVCMOS33", PULLMODE="UP", DRIVE="4" *)
      TRELLIS_IO #( .DIR("BIDIR") ) gpio_inst ( .B(gpio_pins[g]), .I(gpio_o[g]), .O(gpio_i[g]), .T(gpio_t[g]) );
    end
  endgenerate

  `make_mem_signals(led);
  led_controller #(
    .CLKFREQ(CLKFREQ)
  ) led_controller_inst (
    `connect_mem_signals(led),
    .led_pin(wsled),
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
      `memory_map_device(32'h20000000, led);
      `memory_map_device(32'h30000000, stack_ram);
      `memory_map_device(32'h50000000, jtag);
      `memory_map_device(32'h60000000, clock);
      `memory_map_device(32'h70000000, tactile);
      `memory_map_device(32'h10000000, gpio);
      `memory_map_device(32'h40000000, thermometer);
  end

endmodule
