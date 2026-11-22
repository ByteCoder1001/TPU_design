`timescale 1ns / 1ps

module axil_csr #(
    parameter N          = 8,
    parameter MAX_R      = 256,
    parameter WBUF_BYTES = 131072,
    parameter ABUF_BYTES = 65536
)(
    input  wire        clk,
    input  wire        rstn,

    input  wire [5:0]  awaddr,
    input  wire        awvalid,
    output reg         awready,
    input  wire [31:0] wdata,
    input  wire        wvalid,
    output reg         wready,
    output wire [1:0]  bresp,
    output reg         bvalid,
    input  wire        bready,
    input  wire [5:0]  araddr,
    input  wire        arvalid,
    output reg         arready,
    output reg  [31:0] rdata,
    output wire [1:0]  rresp,
    output reg         rvalid,
    input  wire        rready,

    output reg         start_pulse,
    output reg  [31:0] a_addr, w_addr, c_addr, lda, ldw, ldc,
    output reg  [15:0] m, k, nc, r,
    input  wire        busy,
    input  wire        done_pulse,
    input  wire        mm2s_err,
    input  wire        s2mm_err,
    output reg         done_flag
);

    localparam [15:0] MAXR16 = MAX_R;
    localparam [7:0]  P8     = N;
    localparam [31:0] WB32   = WBUF_BYTES;
    localparam [31:0] AB32   = ABUF_BYTES;

    assign bresp = 2'b00;
    assign rresp = 2'b00;

    reg [31:0] cycles;

    always @(posedge clk) begin
        if (!rstn) begin
            awready <= 1'b0; wready <= 1'b0; bvalid <= 1'b0; start_pulse <= 1'b0;
            a_addr <= 0; w_addr <= 0; c_addr <= 0; lda <= 0; ldw <= 0; ldc <= 0;
            m <= 0; k <= 0; nc <= 0; r <= 0;
        end else begin
            start_pulse <= 1'b0;
            awready     <= 1'b0;
            wready      <= 1'b0;
            if (awvalid && wvalid && !awready && !bvalid) begin
                awready <= 1'b1;
                wready  <= 1'b1;
                bvalid  <= 1'b1;
                case (awaddr[5:2])
                    4'd0:  start_pulse <= wdata[0] & ~busy;
                    4'd2:  a_addr <= wdata;
                    4'd3:  w_addr <= wdata;
                    4'd4:  c_addr <= wdata;
                    4'd5:  lda    <= wdata;
                    4'd6:  ldw    <= wdata;
                    4'd7:  ldc    <= wdata;
                    4'd8:  m      <= wdata[15:0];
                    4'd9:  k      <= wdata[15:0];
                    4'd10: nc     <= wdata[15:0];
                    4'd11: r      <= wdata[15:0];
                    default: ;
                endcase
            end
            if (bvalid && bready)
                bvalid <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            done_flag <= 1'b0;
            cycles    <= 32'd0;
        end else if (start_pulse) begin
            done_flag <= 1'b0;
            cycles    <= 32'd0;
        end else begin
            if (busy)       cycles    <= cycles + 32'd1;
            if (done_pulse) done_flag <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            arready <= 1'b0; rvalid <= 1'b0; rdata <= 32'd0;
        end else begin
            arready <= 1'b0;
            if (arvalid && !arready && !rvalid) begin
                arready <= 1'b1;
                rvalid  <= 1'b1;
                case (araddr[5:2])
                    4'd1:  rdata <= {28'd0, s2mm_err, mm2s_err, done_flag, busy};
                    4'd2:  rdata <= a_addr;
                    4'd3:  rdata <= w_addr;
                    4'd4:  rdata <= c_addr;
                    4'd5:  rdata <= lda;
                    4'd6:  rdata <= ldw;
                    4'd7:  rdata <= ldc;
                    4'd8:  rdata <= {16'd0, m};
                    4'd9:  rdata <= {16'd0, k};
                    4'd10: rdata <= {16'd0, nc};
                    4'd11: rdata <= {16'd0, r};
                    4'd12: rdata <= cycles;
                    4'd13: rdata <= {MAXR16, P8, 8'h03};
                    4'd14: rdata <= WB32;
                    4'd15: rdata <= AB32;
                    default: rdata <= 32'd0;
                endcase
            end
            if (rvalid && rready)
                rvalid <= 1'b0;
        end
    end

endmodule
