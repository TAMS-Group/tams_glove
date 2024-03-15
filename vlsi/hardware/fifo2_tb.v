// 2023-2024 Philipp Ruppel

`include "../3rdparty/afifo.v"

module top ( );

    localparam WIDTH = 8;

    reg clk = 0;

	wire wr;
    wire [WIDTH-1:0] wdata;
    wire wfull;

    wire rd;
    wire [WIDTH-1:0] rdata;
    wire rempty;

    reg nreset = 0;

    afifo #(
        .DSIZE(WIDTH)
    ) fifo_inst (

        .i_wclk(clk),
        .i_wrst_n(nreset),
        .i_wr(wr),
        .i_wdata(wdata),
        .o_wfull(wfull),

        .i_rclk(clk),
        .i_rrst_n(nreset),
        .i_rd(rd),
        .o_rdata(rdata),
        .o_rempty(rempty)
    );

    reg rand1 = 0;
    always @(posedge clk) begin
        rand1 <= ($random % 20 == 0);
    end

    reg [WIDTH-1:0] i_cntr = 0;
    always @(posedge clk) begin
        if (wr) begin
            i_cntr <= i_cntr + 1;
        end
    end

    assign wr = (rand1 && !wfull);
    assign wdata = i_cntr;



    reg rand2 = 0;
    always @(posedge clk) begin
        rand2 <= ($random % 20 == 0);
    end

    assign rd = (!rempty && rand2);

    reg [WIDTH-1:0] o_cntr = 0;
    always @(posedge clk) begin
        if (rd) begin
            $display("%3d %3d", o_cntr, rdata);
            o_cntr <= o_cntr + 1;
        end
    end

    integer i;
    initial begin
        #100;
        nreset <= 1;
        #100;
        for (i = 0; i < 1000000; i = i + 1) begin
            // $display(i);
            clk <= ~clk;
            #1000;
        end
    end

endmodule
