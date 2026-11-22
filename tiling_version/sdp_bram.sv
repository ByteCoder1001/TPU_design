`timescale 1ns / 1ps

module sdp_bram #(
    parameter W     = 64,
    parameter DEPTH = 1024,
    parameter AW    = $clog2(DEPTH)
)(
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [W-1:0]  wdata,
    input  wire [AW-1:0] raddr,
    output reg  [W-1:0]  rdata
);

    (* ram_style = "block" *)
    reg [W-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
    end

    always @(posedge clk) begin
        rdata <= mem[raddr];
    end

endmodule
