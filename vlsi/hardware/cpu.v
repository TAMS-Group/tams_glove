// 2021-2024 Philipp Ruppel

`include "../3rdparty/picorv32/picorv32.v"

module cpu #(
  parameter STACK_START = 0,
  parameter PROGRAM_START = 0
) (
  input wire clk,
  output wire mem_valid,
  output wire [31:0] mem_addr,
  output wire [3:0] mem_wstrb,
  output wire [31:0] mem_wdata,
  input wire [31:0] mem_rdata,
  input wire mem_ready,
  input wire mem_error
);

  reg [9:0] resetn_counter = 0;
  wire resetn = resetn_counter[9];
  always @(posedge clk) begin
      if (!resetn) begin
          resetn_counter <= resetn_counter + 1;
      end
  end

  reg mem_trap = 0;
  always @(posedge clk) begin
    if (mem_valid && mem_error) begin
      mem_trap <= 1;
    end
  end

  wire core_mem_valid;
  wire core_trap;
  picorv32 #(
    .PROGADDR_RESET(PROGRAM_START),
    .LATCHED_MEM_RDATA(0),
    .BARREL_SHIFTER(0),
    .COMPRESSED_ISA(0),
    .ENABLE_COUNTERS(0),
    .ENABLE_COUNTERS64(0),
    .ENABLE_REGS_16_31(1),
    .ENABLE_REGS_DUALPORT(1),
    .TWO_STAGE_SHIFT(1),
    .ENABLE_MUL(0),
    .ENABLE_DIV(0),
    .ENABLE_IRQ(0),
    .ENABLE_IRQ_QREGS(0),
    .TWO_CYCLE_COMPARE(0),
    .TWO_CYCLE_ALU(0),
    .CATCH_ILLINSN(0),
    .CATCH_MISALIGN(0),
    .STACKADDR(STACK_START)
  ) cpu (
    .clk(clk),
    .resetn(resetn),
    .mem_ready(mem_ready && !mem_trap),
    .mem_valid(core_mem_valid),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(mem_rdata),
    .irq(0),
    .trap(core_trap)
  );
  assign mem_valid = (core_mem_valid && resetn);

endmodule
