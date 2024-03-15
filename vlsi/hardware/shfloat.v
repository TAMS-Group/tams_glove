// 2023-2024 Philipp Ruppel

module shfloat #(
    parameter INPUT_DEPTH = 32,
    parameter MANTISSA_DEPTH = 12,
    parameter EXPONENT_DEPTH = 8,
    parameter PACKED_DEPTH = 32,
    parameter INDEX_DEPTH = 32
) (
    input wire clk,
    input wire [INDEX_DEPTH-1:0] in_index,
    input wire [INPUT_DEPTH-1:0] in_value_i,
    input wire [INPUT_DEPTH-1:0] in_value_q,
    input wire in_strobe,
    output reg [INDEX_DEPTH-1:0] out_index,
    output reg [PACKED_DEPTH-1:0] out_pack,
    output reg out_strobe,
    output reg [31:0] out_dbg_exp,
    output reg [31:0] out_dbg_i,
    output reg [31:0] out_dbg_q
);

    initial begin
        out_index = 0;
        out_pack = 0;
        out_strobe = 0;
    end

    reg [INDEX_DEPTH-1:0] a_index = 0;
    reg signed [INPUT_DEPTH-1:0] a_value_i = 0;
    reg signed [INPUT_DEPTH-1:0] a_value_q = 0;
    reg signed [INPUT_DEPTH-1:0] a_abs_i = 0;
    reg signed [INPUT_DEPTH-1:0] a_abs_q = 0;
    reg a_strobe = 0;
    always @(posedge clk) begin
        a_index <= in_index;
        a_value_i <= in_value_i;
        a_value_q <= in_value_q;
        a_abs_i <= (in_value_i[INPUT_DEPTH-1] ? ~in_value_i : in_value_i);
        a_abs_q <= (in_value_q[INPUT_DEPTH-1] ? ~in_value_q : in_value_q);
        a_strobe <= in_strobe;
    end

    reg [INDEX_DEPTH-1:0] b_index = 0;
    reg signed [INPUT_DEPTH-1:0] b_value_i = 0;
    reg signed [INPUT_DEPTH-1:0] b_value_q = 0;
    reg b_strobe = 0;
    always @(posedge clk) begin
        b_index <= a_index;
        b_value_i <= a_value_i;
        b_value_q <= a_value_q;
        b_strobe <= a_strobe;
    end

    reg [EXPONENT_DEPTH-1:0] b_exp;
    integer it;

    /* verilator lint_off WIDTH */
    always @(posedge clk) begin
        for (it = INPUT_DEPTH - 1; it >= 0; it = it - 1) begin
            if (((a_abs_i >> it)) == 0 && ((a_abs_q >> it) == 0)) begin
                b_exp <= 31 - it;
            end
        end
    end

    always @(posedge clk) begin
        out_index <= b_index;
        out_pack <= (b_exp | (((b_value_i << b_exp) & 32'hfff00000)) | (((b_value_q << b_exp) & 32'hfff00000) >> MANTISSA_DEPTH));
        out_strobe <= b_strobe;
        out_dbg_exp <= b_exp;
        out_dbg_i <= b_value_i;
        out_dbg_q <= b_value_q;
    end

endmodule