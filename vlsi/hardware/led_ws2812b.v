// 2021-2024 Philipp Ruppel

module ws2812b #(
  parameter DATABITS = 24,
  parameter CLKFREQ = 10000000
) (
  input wire clk,
  input wire [DATABITS-1:0] color,
  output reg pin
);

  localparam T0H = $floor(CLKFREQ * 0.35 / 1000000.0 + 0.5);
  localparam T1H = $floor(CLKFREQ * 0.7 / 1000000.0 + 0.5);
  localparam T0L = $floor(CLKFREQ * 0.8 / 1000000.0 + 0.5);
  localparam T1L = $floor(CLKFREQ * 0.6 / 1000000.0 + 0.5);
  localparam TRES = $floor(CLKFREQ * 100.0 / 1000000.0 + 0.5);

  reg [DATABITS-1:0] colorbuf = 0;

  localparam STATE_BUFFER = 0;
  localparam STATE_HIGH = 1;
  localparam STATE_LOW = 2;
  localparam STATE_DELAY = 3;
  localparam STATE_WAIT = 4;
  reg [3:0] state = 0;

  reg [31:0] delaycount = 0;

  reg [15:0] txbit = 0;

  always @(posedge clk) begin

    if (delaycount == 0) begin

      case (state)

      STATE_BUFFER: begin
        colorbuf <= color;
        state <= STATE_HIGH;
        txbit <= 0;
      end

      STATE_HIGH: begin
        pin <= 1;
        if (colorbuf[DATABITS-1])
          delaycount <= T1H;
        else
          delaycount <= T0H;
        state <= STATE_LOW;
      end

      STATE_LOW: begin
        pin <= 0;
        if (txbit == DATABITS-1) begin
          state <= STATE_DELAY;
        end else begin
          state <= STATE_HIGH;
          txbit <= txbit + 1;
        end
        if (colorbuf[DATABITS-1])
          delaycount <= T1L;
        else
          delaycount <= T0L;
        colorbuf <= (colorbuf << 1);
      end

      STATE_DELAY: begin
        delaycount <= TRES;
        state <= STATE_WAIT;
        pin <= 1;
      end

      STATE_WAIT: begin
        delaycount <= TRES;
        state <= STATE_BUFFER;
        pin <= 0;
      end

      endcase

    end else begin

      delaycount <= delaycount - 1;

    end

  end

endmodule
