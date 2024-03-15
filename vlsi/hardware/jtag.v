// 2021-2024 Philipp Ruppel

`include "utils.v"
`include "fifo.v"

module jtag (
  `make_mem_ports()
);

  wire jtag_tck;
  wire jtag_tdi;
  wire jtag_shift;
  wire jtag_update;
  wire jtag_rstn;
  wire jtag_ce1;
  wire jtag_ce2;
  wire jtag_rti1;
  wire jtag_rti2;
  wire jtag_tdo1;
  wire jtag_tdo2;
  JTAGG jtagg_inst(
    .JTCK(jtag_tck),
    .JTDI(jtag_tdi),
    .JSHIFT(jtag_shift),
    .JUPDATE(jtag_update),
    .JRSTN(jtag_rstn),
    .JCE1(jtag_ce1),
    .JCE2(jtag_ce2),
    .JRTI1(jtag_rti1),
    .JRTI2(jtag_rti2),
    .JTDO1(jtag_tdo1),
    .JTDO2(jtag_tdo2)
  );

  reg fifo_in_shift = 0;
  reg [7:0] fifo_in_data = 0;
  reg fifo_out_pop = 0;
  wire [7:0] fifo_out_data;
  wire fifo_out_nempty;

  simple_fifo #(
    .WIDTH(8),
    .DEPTH(16 * 1024),
  ) fifo_inst (
    .clk(clk),
    .in_shift(fifo_in_shift),
    .in_data(fifo_in_data),
  	.out_pop(fifo_out_pop),
  	.out_data(fifo_out_data),
  	.out_nempty(fifo_out_nempty),
  );

  always @(posedge clk) begin
    `mem_begin();
    fifo_in_shift <= 0;
    fifo_in_data <= 0;
    if (mem_valid && mem_addr == 0 && mem_wstrb == 4'b1111 && !mem_ready) begin
      fifo_in_shift <= 1;
      fifo_in_data <= mem_wdata[7:0];
      mem_ready <= 1;
      mem_error <= 0;
    end
  end

  reg jtag_tck_buf = 0;
  reg jtag_tck_prev = 0;
  reg [8:0] jtag_data = 0;
  reg [3:0] jtag_index = 0;
  reg jtag_tdo_buf = 0;
  reg fifo_pop_buf = 0;

  always @(posedge clk) begin

    fifo_out_pop <= 0;

    jtag_tck_buf <= jtag_tck;
    jtag_tck_prev <= jtag_tck_buf;
    if (jtag_tck_buf && !jtag_tck_prev) begin

      if (jtag_ce1 && jtag_shift) begin

        jtag_data <= (jtag_data >> 1);

        if (jtag_index == 8) begin
          jtag_index <= 0;
        end else begin
          jtag_index <= jtag_index + 1;
        end

        if (jtag_index == 0) begin
          if (fifo_out_nempty) begin
            fifo_pop_buf <= 1;
            jtag_data <= { 1'b1, fifo_out_data };
          end else begin
            jtag_data <= 0;
          end
        end

        if (fifo_pop_buf) begin
          fifo_pop_buf <= 0;
          fifo_out_pop <= 1;
        end

      end

      if (jtag_update) begin
        jtag_index <= 0;
        fifo_pop_buf <= 0;
      end

    end

    jtag_tdo_buf <= jtag_data[0];

  end

  assign jtag_tdo1 = jtag_tdo_buf;


endmodule
