// 2021-2024 Philipp Ruppel

`define reg_8(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        mem_rdata <= register; \
        if (mem_wstrb[0]) register <= mem_wdata[7:0]; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_8_w(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        if (mem_wstrb[0]) register <= mem_wdata[7:0]; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_8_r(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        mem_rdata <= {24'b0, register}; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_16_r(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        mem_rdata <= {16'b0, register}; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_1_r(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        mem_rdata <= {31'b0, register}; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_1_w(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        if (mem_wstrb[0]) register <= mem_wdata[0]; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_1(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        mem_rdata <= {31'b0, register}; \
        if (mem_wstrb[0]) register <= mem_wdata[0]; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_32(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr) begin \
        mem_rdata <= register; \
        if (mem_wstrb && mem_wstrb == 4'b1111) register <= mem_wdata; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_32_w(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr && mem_wstrb == 4'b1111) begin \
        register <= mem_wdata; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define reg_32_r(addr, register) \
    if (mem_valid && !mem_ready && mem_addr == addr && mem_wstrb == 4'b0000) begin \
        mem_rdata <= register; \
        mem_ready <= 1; \
        mem_error <= 0; \
    end

`define memory_map_device(addr, name) \
    if (mem_valid && mem_device == addr) begin \
        name``_mem_wstrb = mem_wstrb; \
        mem_rdata = name``_mem_rdata; \
        name``_mem_valid = mem_valid; \
        mem_ready = name``_mem_ready; \
        mem_error = name``_mem_error; \
    end else begin \
        name``_mem_wstrb = 4'b000; \
        name``_mem_valid = 0; \
    end

`define make_mem_ports() \
    input wire clk, \
    input wire mem_valid, \
    input wire [31:0] mem_addr, \
    input wire [3:0] mem_wstrb, \
    input wire [31:0] mem_wdata, \
    output reg [31:0] mem_rdata, \
    output reg mem_ready, \
    output reg mem_error

`define make_mem_signals(name) \
    wire name``_mem_valid; \
    wire [3:0] name``_mem_wstrb; \
    wire [31:0] name``_mem_rdata; \
    wire name``_mem_ready; \
    wire name``_mem_error;

`define connect_mem_signals(name) \
    .clk(clk), \
    .mem_valid(name``_mem_valid), \
    .mem_addr(dev_mem_addr), \
    .mem_wdata(mem_wdata), \
    .mem_wstrb(name``_mem_wstrb), \
    .mem_rdata(name``_mem_rdata), \
    .mem_ready(name``_mem_ready), \
    .mem_error(name``_mem_error)

`define forward_mem_signals() \
    .clk(clk), \
    .mem_valid(mem_valid), \
    .mem_addr(mem_addr), \
    .mem_wdata(mem_wdata), \
    .mem_wstrb(mem_wstrb), \
    .mem_rdata(mem_rdata), \
    .mem_ready(mem_ready), \
    .mem_error(mem_error)

`define mem_begin() \
    mem_ready <= 0; \
    mem_error <= mem_valid;
