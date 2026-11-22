`timescale 1ns / 1ps


module acc_flush #(
    parameter N         = 8,
    parameter ACC_WIDTH = 32,
    parameter MAX_R     = 256,
    parameter OUT_W     = 64
)(
    input  wire clk,
    input  wire rst,
    input  wire job_start,

    input  wire                     we,
    input  wire                     first,
    input  wire                     wbank,
    input  wire [15:0]              wrow,
    input  wire [N*ACC_WIDTH-1:0]   din,

    input  wire                     fl_start,
    input  wire                     fl_bank,
    input  wire [15:0]              fl_rows,
    input  wire [31:0]              fl_addr,
    input  wire [31:0]              ldc,
    output reg                      fl_busy,
    output wire                     wr_idle,
    output reg                      err,

    output wire [71:0]              m_axis_cmd_tdata,
    output wire                     m_axis_cmd_tvalid,
    input  wire                     m_axis_cmd_tready,

    output wire [OUT_W-1:0]         m_axis_tdata,
    output wire [OUT_W/8-1:0]       m_axis_tkeep,
    output wire                     m_axis_tvalid,
    input  wire                     m_axis_tready,
    output wire                     m_axis_tlast,

    input  wire [7:0]               s_axis_sts_tdata,
    input  wire                     s_axis_sts_tvalid,
    output wire                     s_axis_sts_tready
);

    localparam RW  = N*ACC_WIDTH;
    localparam RAW = $clog2(MAX_R);
    localparam NB  = RW / OUT_W;             
    localparam [22:0] ROW_BYTES = RW / 8;

    wire [15:0]   rmw_row = we ? (wrow + 16'd1) : wrow;
    reg  [RAW-1:0] fraddr;
    wire [RW-1:0] rdata [0:1];
    wire [RW-1:0] old = rdata[wbank];
    wire [RW-1:0] nxt;

    genvar b;
    generate
        for (b = 0; b < 2; b = b + 1) begin : bank
            sdp_bram #(.W(RW), .DEPTH(MAX_R)) mem (
                .clk  (clk),
                .we   (we && (wbank == b)),
                .waddr(wrow[RAW-1:0]),
                .wdata(nxt),
                .raddr((wbank == b) ? rmw_row[RAW-1:0] : fraddr),
                .rdata(rdata[b])
            );
        end
    endgenerate

    genvar l;
    generate
        for (l = 0; l < N; l = l + 1) begin : lane
            assign nxt[l*ACC_WIDTH +: ACC_WIDTH] = first ?
                din[l*ACC_WIDTH +: ACC_WIDTH] :
                old[l*ACC_WIDTH +: ACC_WIDTH] + din[l*ACC_WIDTH +: ACC_WIDTH];
        end
    endgenerate

    localparam [2:0] F_IDLE = 3'd0, F_PRE = 3'd1, F_CAP = 3'd2, F_CMD = 3'd3, F_DATA = 3'd4;

    reg [2:0]      fst;
    reg            fbank;
    reg [15:0]     frows, frow;
    reg [31:0]     faddr;
    reg [RW-1:0]   row_reg;
    reg [$clog2(NB+1)-1:0] beat;

    assign m_axis_cmd_tvalid = (fst == F_CMD);
    assign m_axis_cmd_tdata  = {4'h0, 4'h0, faddr, 1'b0, 1'b1, 6'd0, 1'b1, ROW_BYTES};

    assign m_axis_tvalid = (fst == F_DATA);
    assign m_axis_tdata  = row_reg[beat*OUT_W +: OUT_W];
    assign m_axis_tkeep  = {(OUT_W/8){1'b1}};
    assign m_axis_tlast  = (beat == NB-1);

    always @(posedge clk) begin
        if (rst) begin
            fst     <= F_IDLE;
            fl_busy <= 1'b0;
            fbank   <= 1'b0;
            frows   <= 16'd0;
            frow    <= 16'd0;
            faddr   <= 32'd0;
            fraddr  <= {RAW{1'b0}};
            beat    <= 0;
            row_reg <= {RW{1'b0}};
        end else begin
            case (fst)
                F_IDLE: if (fl_start) begin
                    fl_busy <= 1'b1;
                    fbank   <= fl_bank;
                    frows   <= fl_rows;
                    frow    <= 16'd0;
                    faddr   <= fl_addr;
                    fraddr  <= {RAW{1'b0}};
                    fst     <= F_PRE;
                end
                F_PRE:  fst <= F_CAP;                
                F_CAP: begin
                    row_reg <= rdata[fbank];
                    fraddr  <= fraddr + 1'b1;         
                    fst     <= F_CMD;
                end
                F_CMD: if (m_axis_cmd_tready) begin
                    beat <= 0;
                    fst  <= F_DATA;
                end
                F_DATA: if (m_axis_tready) begin
                    if (beat == NB-1) begin
                        faddr <= faddr + ldc;
                        if (frow == frows - 16'd1) begin
                            fl_busy <= 1'b0;
                            fst     <= F_IDLE;
                        end else begin
                            frow    <= frow + 16'd1;
                            row_reg <= rdata[fbank];
                            fraddr  <= fraddr + 1'b1;
                            fst     <= F_CMD;
                        end
                    end else
                        beat <= beat + 1'b1;
                end
                default: fst <= F_IDLE;
            endcase
        end
    end

    reg [15:0] cmd_cnt, sts_cnt;
    assign s_axis_sts_tready = 1'b1;
    assign wr_idle = (cmd_cnt == sts_cnt);

    always @(posedge clk) begin
        if (rst) begin
            cmd_cnt <= 16'd0;
            sts_cnt <= 16'd0;
            err     <= 1'b0;
        end else begin
            if (m_axis_cmd_tvalid && m_axis_cmd_tready) cmd_cnt <= cmd_cnt + 16'd1;
            if (s_axis_sts_tvalid)                      sts_cnt <= sts_cnt + 16'd1;
            if (job_start)
                err <= 1'b0;
            else if (s_axis_sts_tvalid && |s_axis_sts_tdata[6:4])
                err <= 1'b1;
        end
    end

endmodule
