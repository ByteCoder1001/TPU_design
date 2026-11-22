`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// tpu_accelerator_top : v2 tiled weight-stationary TPU (single N x N array)
//   * AXI4-Lite slave  -> CSR (start/done/busy, matrix pointers, dims)
//   * AXI-Stream master/slave pairs -> one AXI DataMover (MM2S + S2MM)
//   * W kept fully on-chip (wbuf), A double-buffered per row-block (abuf),
//     C accumulated on-chip (ping-pong acc_flush) and written back row by row.
// Replaces: 3x AXI BRAM controllers + AXI GPIO of v1.
// ---------------------------------------------------------------------------
(* X_CORE_INFO = "tpu_accelerator_top" *)
module tpu_accelerator_top #(
    parameter N          = 8,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32,
    parameter WB_AW      = 14,   // wbuf depth = 2^WB_AW 64-bit words (128 KB @ 14)
    parameter AB_AW      = 13,   // abuf depth per bank = 2^AB_AW words (64 KB/bank)
    parameter MAX_R      = 256   // max rows per A block (accumulator depth)
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:m_axis_mm2s_cmd:s_axis_mm2s:s_axis_mm2s_sts:m_axis_s2mm_cmd:m_axis_s2mm:s_axis_s2mm_sts, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    // AXI4-Lite slave (CSR)
    input  wire [5:0]  s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [5:0]  s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,

    // DataMover MM2S (read DDR -> wbuf/abuf)
    output wire [71:0] m_axis_mm2s_cmd_tdata,
    output wire        m_axis_mm2s_cmd_tvalid,
    input  wire        m_axis_mm2s_cmd_tready,
    input  wire [63:0] s_axis_mm2s_tdata,
    input  wire [7:0]  s_axis_mm2s_tkeep,
    input  wire        s_axis_mm2s_tlast,
    input  wire        s_axis_mm2s_tvalid,
    output wire        s_axis_mm2s_tready,
    input  wire [7:0]  s_axis_mm2s_sts_tdata,
    input  wire        s_axis_mm2s_sts_tvalid,
    output wire        s_axis_mm2s_sts_tready,

    // DataMover S2MM (acc -> DDR)
    output wire [71:0] m_axis_s2mm_cmd_tdata,
    output wire        m_axis_s2mm_cmd_tvalid,
    input  wire        m_axis_s2mm_cmd_tready,
    output wire [63:0] m_axis_s2mm_tdata,
    output wire [7:0]  m_axis_s2mm_tkeep,
    output wire        m_axis_s2mm_tvalid,
    input  wire        m_axis_s2mm_tready,
    output wire        m_axis_s2mm_tlast,
    input  wire [7:0]  s_axis_s2mm_sts_tdata,
    input  wire        s_axis_s2mm_sts_tvalid,
    output wire        s_axis_s2mm_sts_tready,

    output wire        irq
);
    localparam WBUF_BYTES = (1 << WB_AW) * 8;
    localparam ABUF_BYTES = (1 << AB_AW) * 8;   // per bank
    localparam PW = N*DATA_WIDTH;               // one buffer word = one tile row

    wire rst = ~aresetn;

    // ---------------- CSR ----------------
    wire        start_pulse, busy, done_pulse, done_flag;
    wire [31:0] a_addr, w_addr, c_addr, lda, ldw, ldc;
    wire [15:0] m, k, nc, r;
    wire        mm2s_err, s2mm_err;

    axil_csr #(.N(N), .MAX_R(MAX_R), .WBUF_BYTES(WBUF_BYTES), .ABUF_BYTES(ABUF_BYTES)) u_csr (
        .clk(clk), .rstn(aresetn),
        .awaddr(s_axi_awaddr), .awvalid(s_axi_awvalid), .awready(s_axi_awready),
        .wdata(s_axi_wdata),   .wvalid(s_axi_wvalid),   .wready(s_axi_wready),
        .bresp(s_axi_bresp),   .bvalid(s_axi_bvalid),   .bready(s_axi_bready),
        .araddr(s_axi_araddr), .arvalid(s_axi_arvalid), .arready(s_axi_arready),
        .rdata(s_axi_rdata),   .rresp(s_axi_rresp),     .rvalid(s_axi_rvalid), .rready(s_axi_rready),
        .start_pulse(start_pulse),
        .a_addr(a_addr), .w_addr(w_addr), .c_addr(c_addr),
        .lda(lda), .ldw(ldw), .ldc(ldc),
        .m(m), .k(k), .nc(nc), .r(r),
        .busy(busy), .done_pulse(done_pulse),
        .mm2s_err(mm2s_err), .s2mm_err(s2mm_err), .done_flag(done_flag)
    );
    assign irq = done_flag;

    // ---------------- tile sequencer ----------------
    wire        f_start, f_sel_a, f_bank, f_busy;
    wire [31:0] f_base, f_stride;
    wire [15:0] f_rows, f_row_bytes;
    wire [WB_AW-1:0] wbuf_raddr;
    wire [AB_AW:0]   abuf_raddr;
    wire        load_weight, core_valid;
    wire        acc_we, acc_first, acc_bank;
    wire [15:0] acc_wrow;
    wire        fl_start, fl_bank, fl_busy, wr_idle;
    wire [15:0] fl_rows;
    wire [31:0] fl_addr;

    tpu_fsm #(.N(N), .WB_AW(WB_AW), .AB_AW(AB_AW)) u_ctrl (
        .clk(clk), .rst(rst),
        .start(start_pulse),
        .cfg_a_addr(a_addr), .cfg_w_addr(w_addr), .cfg_c_addr(c_addr),
        .cfg_lda(lda), .cfg_ldw(ldw), .cfg_ldc(ldc),
        .cfg_m(m), .cfg_k(k), .cfg_nc(nc), .cfg_r(r),
        .busy(busy), .done_pulse(done_pulse),
        .f_start(f_start), .f_base(f_base), .f_rows(f_rows), .f_row_bytes(f_row_bytes),
        .f_stride(f_stride), .f_sel_a(f_sel_a), .f_bank(f_bank), .f_busy(f_busy),
        .wbuf_raddr(wbuf_raddr), .abuf_raddr(abuf_raddr),
        .load_weight(load_weight), .core_valid(core_valid),
        .acc_we(acc_we), .acc_first(acc_first), .acc_bank(acc_bank), .acc_wrow(acc_wrow),
        .fl_start(fl_start), .fl_bank(fl_bank), .fl_rows(fl_rows), .fl_addr(fl_addr),
        .fl_busy(fl_busy), .wr_idle(wr_idle)
    );

    // ---------------- MM2S front end (W and A share one stream) ----------------
    wire          wbuf_we, abuf_we, abuf_bank;
    wire [15:0]   buf_waddr;
    wire [PW-1:0] buf_wdata;

    dm_fetch #(.SW(PW)) u_fetch (
        .clk(clk), .rst(rst), .job_start(start_pulse),
        .start(f_start), .base(f_base), .rows(f_rows), .row_bytes(f_row_bytes),
        .stride(f_stride), .sel_a(f_sel_a), .bank(f_bank),
        .busy(f_busy), .err(mm2s_err),
        .m_axis_cmd_tdata(m_axis_mm2s_cmd_tdata),
        .m_axis_cmd_tvalid(m_axis_mm2s_cmd_tvalid),
        .m_axis_cmd_tready(m_axis_mm2s_cmd_tready),
        .s_axis_tdata(s_axis_mm2s_tdata), .s_axis_tvalid(s_axis_mm2s_tvalid),
        .s_axis_tready(s_axis_mm2s_tready),
        .s_axis_sts_tdata(s_axis_mm2s_sts_tdata), .s_axis_sts_tvalid(s_axis_mm2s_sts_tvalid),
        .s_axis_sts_tready(s_axis_mm2s_sts_tready),
        .wbuf_we(wbuf_we), .abuf_we(abuf_we), .abuf_bank(abuf_bank),
        .buf_waddr(buf_waddr), .buf_wdata(buf_wdata)
    );

    // ---------------- on-chip buffers ----------------
    wire [PW-1:0] wbuf_rdata, abuf_rdata;

    // v1 dual_port_bram reused. Port A = DMA write (word addressed), port B = array read.
    dual_port_bram #(.DATA_WIDTH(PW), .ADDR_WIDTH(WB_AW), .BYTE_ADDR(0)) weight_bram (
        .clk(clk),
        .ena(wbuf_we), .wea(wbuf_we), .addra(buf_waddr[WB_AW-1:0]), .dina(buf_wdata), .douta(),
        .enb(1'b1),    .web(1'b0),    .addrb(wbuf_raddr), .dinb({PW{1'b0}}),          .doutb(wbuf_rdata)
    );
    dual_port_bram #(.DATA_WIDTH(PW), .ADDR_WIDTH(AB_AW+1), .BYTE_ADDR(0)) act_bram (
        .clk(clk),
        .ena(abuf_we), .wea(abuf_we), .addra({abuf_bank, buf_waddr[AB_AW-1:0]}), .dina(buf_wdata), .douta(),
        .enb(1'b1),    .web(1'b0),    .addrb(abuf_raddr), .dinb({PW{1'b0}}),                      .doutb(abuf_rdata)
    );

    // ---------------- compute ----------------
    wire [N-1:0][DATA_WIDTH-1:0] act_vec = abuf_rdata;
    wire [N-1:0][DATA_WIDTH-1:0] wt_vec  = wbuf_rdata;
    wire [N-1:0][ACC_WIDTH-1:0]  psum_vec;

    compute_core #(.N(N), .DATA_WIDTH(DATA_WIDTH), .ACC_WIDTH(ACC_WIDTH)) u_core (
        .clk(clk), .reset(rst),
        .load_weight(load_weight), .valid(core_valid),
        .act_in(act_vec), .weight_in(wt_vec), .psum_out(psum_vec)
    );

    // ---------------- accumulator + S2MM back end ----------------
    acc_flush #(.N(N), .ACC_WIDTH(ACC_WIDTH), .MAX_R(MAX_R), .OUT_W(64)) u_acc (
        .clk(clk), .rst(rst), .job_start(start_pulse),
        .we(acc_we), .first(acc_first), .wbank(acc_bank), .wrow(acc_wrow), .din(psum_vec),
        .fl_start(fl_start), .fl_bank(fl_bank), .fl_rows(fl_rows), .fl_addr(fl_addr),
        .ldc(ldc), .fl_busy(fl_busy), .wr_idle(wr_idle), .err(s2mm_err),
        .m_axis_cmd_tdata(m_axis_s2mm_cmd_tdata),
        .m_axis_cmd_tvalid(m_axis_s2mm_cmd_tvalid),
        .m_axis_cmd_tready(m_axis_s2mm_cmd_tready),
        .m_axis_tdata(m_axis_s2mm_tdata), .m_axis_tkeep(m_axis_s2mm_tkeep),
        .m_axis_tvalid(m_axis_s2mm_tvalid), .m_axis_tready(m_axis_s2mm_tready),
        .m_axis_tlast(m_axis_s2mm_tlast),
        .s_axis_sts_tdata(s_axis_s2mm_sts_tdata), .s_axis_sts_tvalid(s_axis_s2mm_sts_tvalid),
        .s_axis_sts_tready(s_axis_s2mm_sts_tready)
    );
endmodule
