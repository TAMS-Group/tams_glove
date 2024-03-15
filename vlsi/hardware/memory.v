// 2021-2024 Philipp Ruppel

`include "utils.v"

module memory #(
  parameter MEM_WORDS = 256,
  parameter FILE_NAME = ""
) (
  `make_mem_ports()
);

  reg [31:0] data [0:MEM_WORDS-1];
  initial begin
    if (FILE_NAME != "") begin
      $readmemh(FILE_NAME, data);
    end
  end

  always @(posedge clk) begin
    `mem_begin();
    if (mem_valid && !mem_ready) begin
      mem_ready <= 1;
      mem_error <= 0;
      mem_rdata <= data[mem_addr >> 2];
      if (mem_wstrb[0]) data[mem_addr >> 2][7:0] <= mem_wdata[7:0];
      if (mem_wstrb[1]) data[mem_addr >> 2][15:8] <= mem_wdata[15:8];
      if (mem_wstrb[2]) data[mem_addr >> 2][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) data[mem_addr >> 2][31:24] <= mem_wdata[31:24];
    end
  end

endmodule
