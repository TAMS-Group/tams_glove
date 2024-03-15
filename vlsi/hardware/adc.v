// 2021-2024 Philipp Ruppel

module adc #(
    parameter CHANNELS = 4,
    parameter ADC_BITS = 16
) (
    input wire clk,
    input wire [CHANNELS-1:0] pins_in,
    output wire [CHANNELS-1:0] pins_out,
    output reg [CHANNELS*ADC_BITS-1:0] data
);
  integer i;
  reg [CHANNELS-1:0] regs_in;
  always @(negedge clk) begin
    for (i = 0; i < CHANNELS; i = i + 1) begin
      regs_in[i] <= ~pins_in[i];
    end
  end
  assign pins_out = regs_in;
  always @(posedge clk) begin
    for (i = 0; i < CHANNELS; i = i + 1) begin
        data[i*ADC_BITS+ADC_BITS-1:i*ADC_BITS] <= data[i*ADC_BITS+ADC_BITS-1:i*ADC_BITS] + (regs_in[i] ? 1 : -1);
    end
  end
endmodule
