`timescale 1ns / 1ps

module tpu_fsm #(
    parameter N     = 8,
    parameter WB_AW = 14,      
    parameter AB_AW = 13      
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    input  wire [31:0] cfg_a_addr,
    input  wire [31:0] cfg_w_addr,
    input  wire [31:0] cfg_c_addr,
    input  wire [31:0] cfg_lda,
    input  wire [31:0] cfg_ldw,
    input  wire [31:0] cfg_ldc,
    input  wire [15:0] cfg_m,
    input  wire [15:0] cfg_k,
    input  wire [15:0] cfg_nc,
    input  wire [15:0] cfg_r,
    output wire        busy,
    output reg         done_pulse,

    output reg         f_start,
    output reg  [31:0] f_base,
    output reg  [15:0] f_rows,
    output reg  [15:0] f_row_bytes,
    output reg  [31:0] f_stride,
    output reg         f_sel_a,
    output reg         f_bank,
    input  wire        f_busy,

    output wire [WB_AW-1:0] wbuf_raddr,
    output wire [AB_AW:0]   abuf_raddr,   

    output wire        load_weight,
    output wire        core_valid,

    output wire        acc_we,
    output wire        acc_first,
    output wire        acc_bank,
    output wire [15:0] acc_wrow,

    output reg         fl_start,
    output reg         fl_bank,
    output reg  [15:0] fl_rows,
    output reg  [31:0] fl_addr,
    input  wire        fl_busy,
    input  wire        wr_idle
);

    localparam LOG2P  = $clog2(N);
    localparam PIPE_D = 2*N - 1;

    localparam [3:0] S_IDLE  = 4'd0,
                     S_WF    = 4'd1,   
                     S_ABLK  = 4'd2,  
                     S_SETUP = 4'd3,
                     S_LOAD  = 4'd4,
                     S_FEED  = 4'd5,
                     S_DRAIN = 4'd6,
                     S_FLUSH = 4'd7,
                     S_FIN   = 4'd8;

    reg [3:0]  state;

    reg [15:0] tk, tn, p1tn;
    reg [31:0] blk_a_stride, blk_c_stride;

    reg [31:0] a_next, c_blk;
    reg [15:0] rows_left, pend_rows, rows_blk;
    reg        pend_bank, abank, last_blk;

    reg [15:0] ni, ki, wband, waddr, aaddr, cnt;
    reg        accbank;

    wire [15:0] nrows = (rows_left > cfg_r) ? cfg_r : rows_left;
    wire        fetch_idle = !f_busy && !f_start;

    assign busy       = (state != S_IDLE);
    assign wbuf_raddr = waddr[WB_AW-1:0];
    assign abuf_raddr = {abank, aaddr[AB_AW-1:0]};

    always @(posedge clk) begin
        if (rst) begin
            state      <= S_IDLE;
            done_pulse <= 1'b0;
            f_start    <= 1'b0;
            fl_start   <= 1'b0;
            f_base <= 0; f_rows <= 0; f_row_bytes <= 0; f_stride <= 0; f_sel_a <= 0; f_bank <= 0;
            fl_bank <= 0; fl_rows <= 0; fl_addr <= 0;
            tk <= 0; tn <= 0; p1tn <= 0; blk_a_stride <= 0; blk_c_stride <= 0;
            a_next <= 0; c_blk <= 0; rows_left <= 0; pend_rows <= 0; rows_blk <= 0;
            pend_bank <= 0; abank <= 0; last_blk <= 0;
            ni <= 0; ki <= 0; wband <= 0; waddr <= 0; aaddr <= 0; cnt <= 0; accbank <= 0;
        end else begin
            done_pulse <= 1'b0;
            f_start    <= 1'b0;
            fl_start   <= 1'b0;

            case (state)
         
                S_IDLE: if (start) begin
                    tk           <= cfg_k  >> LOG2P;
                    tn           <= cfg_nc >> LOG2P;
                    p1tn         <= cfg_nc - (cfg_nc >> LOG2P);   
                    blk_a_stride <= cfg_r * cfg_lda;
                    blk_c_stride <= cfg_r * cfg_ldc;
                    a_next       <= cfg_a_addr;
                    c_blk        <= cfg_c_addr;
                    rows_left    <= cfg_m;
                    abank        <= 1'b0;
                    accbank      <= 1'b0;
                    
                    f_start      <= 1'b1;
                    f_base       <= cfg_w_addr;
                    f_rows       <= cfg_k;
                    f_row_bytes  <= cfg_nc;
                    f_stride     <= cfg_ldw;
                    f_sel_a      <= 1'b0;
                    f_bank       <= 1'b0;
                    state        <= S_WF;
                end

                S_WF: if (fetch_idle) begin
                    f_start     <= 1'b1;
                    f_base      <= a_next;
                    f_rows      <= nrows;
                    f_row_bytes <= cfg_k;
                    f_stride    <= cfg_lda;
                    f_sel_a     <= 1'b1;
                    f_bank      <= 1'b0;
                    pend_rows   <= nrows;
                    pend_bank   <= 1'b0;
                    a_next      <= a_next + blk_a_stride;
                    rows_left   <= rows_left - nrows;
                    state       <= S_ABLK;
                end

                S_ABLK: if (fetch_idle) begin
                    rows_blk <= pend_rows;
                    abank    <= pend_bank;
                    ni       <= 16'd0;
                    ki       <= 16'd0;
                    wband    <= 16'd0;
                    if (rows_left != 16'd0) begin
                        f_start     <= 1'b1;
                        f_base      <= a_next;
                        f_rows      <= nrows;
                        f_row_bytes <= cfg_k;
                        f_stride    <= cfg_lda;
                        f_sel_a     <= 1'b1;
                        f_bank      <= ~pend_bank;
                        pend_rows   <= nrows;
                        pend_bank   <= ~pend_bank;
                        a_next      <= a_next + blk_a_stride;
                        rows_left   <= rows_left - nrows;
                        last_blk    <= 1'b0;
                    end else
                        last_blk    <= 1'b1;
                    state <= S_SETUP;
                end

                S_SETUP: begin
                    waddr <= wband + p1tn + ni;     
                    aaddr <= ki;
                    cnt   <= 16'd0;
                    state <= S_LOAD;
                end

                S_LOAD: begin
                    waddr <= waddr - tn;
                    if (cnt == N-1) begin
                        cnt   <= 16'd0;
                        state <= S_FEED;
                    end else
                        cnt <= cnt + 16'd1;
                end

                S_FEED: begin
                    aaddr <= aaddr + tk;
                    if (cnt == rows_blk - 16'd1) begin
                        cnt   <= 16'd0;
                        state <= S_DRAIN;
                    end else
                        cnt <= cnt + 16'd1;
                end

                S_DRAIN: begin
                    if (cnt == PIPE_D-1) begin
                        cnt <= 16'd0;
                        if (ki == tk - 16'd1)
                            state <= S_FLUSH;
                        else begin
                            ki    <= ki + 16'd1;
                            wband <= wband + cfg_nc;
                            state <= S_SETUP;
                        end
                    end else
                        cnt <= cnt + 16'd1;
                end

                S_FLUSH: if (!fl_busy && !fl_start) begin
                    fl_start <= 1'b1;
                    fl_bank  <= accbank;
                    fl_rows  <= rows_blk;
                    fl_addr  <= c_blk + ({16'd0, ni} << (LOG2P + 2)); 
                    accbank  <= ~accbank;
                    ki       <= 16'd0;
                    wband    <= 16'd0;
                    if (ni == tn - 16'd1) begin
                        c_blk <= c_blk + blk_c_stride;
                        state <= last_blk ? S_FIN : S_ABLK;
                    end else begin
                        ni    <= ni + 16'd1;
                        state <= S_SETUP;
                    end
                end

                S_FIN: if (!fl_busy && !fl_start && wr_idle) begin
                    done_pulse <= 1'b1;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    reg              ld_q, fd_q, dr_q, first_q, accbank_q;
    reg [PIPE_D-1:0] pipe;
    reg [15:0]       wrow;

    wire adv = fd_q | dr_q;

    assign load_weight = ld_q;
    assign core_valid  = adv;
    assign acc_we      = adv & pipe[PIPE_D-1];
    assign acc_first   = first_q;
    assign acc_bank    = accbank_q;
    assign acc_wrow    = wrow;

    always @(posedge clk) begin
        if (rst) begin
            ld_q <= 1'b0; fd_q <= 1'b0; dr_q <= 1'b0;
            first_q <= 1'b0; accbank_q <= 1'b0;
            pipe <= {PIPE_D{1'b0}};
            wrow <= 16'd0;
        end else begin
            ld_q      <= (state == S_LOAD);
            fd_q      <= (state == S_FEED);
            dr_q      <= (state == S_DRAIN);
            first_q   <= (ki == 16'd0);
            accbank_q <= accbank;

            if (adv)
                pipe <= {pipe[PIPE_D-2:0], fd_q};

            if (ld_q)
                wrow <= 16'd0;
            else if (acc_we)
                wrow <= wrow + 16'd1;
        end
    end

endmodule
