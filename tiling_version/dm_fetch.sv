`timescale 1ns / 1ps


module dm_fetch #(
    parameter SW = 64              
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        job_start,  

    input  wire        start,
    input  wire [31:0] base,
    input  wire [15:0] rows,
    input  wire [15:0] row_bytes,
    input  wire [31:0] stride,
    input  wire        sel_a,     
    input  wire        bank,       
    output reg         busy,
    output reg         err,
   
    output wire [71:0] m_axis_cmd_tdata,
    output wire        m_axis_cmd_tvalid,
    input  wire        m_axis_cmd_tready,

    input  wire [SW-1:0]   s_axis_tdata,
    input  wire            s_axis_tvalid,
    output wire            s_axis_tready,

    input  wire [7:0]  s_axis_sts_tdata,
    input  wire        s_axis_sts_tvalid,
    output wire        s_axis_sts_tready,

    output wire            wbuf_we,
    output wire            abuf_we,
    output wire            abuf_bank,
    output reg  [15:0]     buf_waddr,
    output wire [SW-1:0]   buf_wdata
);

    localparam BSH = $clog2(SW/8);  

    reg [31:0] addr, step;
    reg [22:0] btt;
    reg [15:0] cmds_left;
    reg [22:0] beats_left;
    reg        r_sel_a, r_bank;

    wire [31:0] total = rows * row_bytes;
    wire        contig = (stride == {16'd0, row_bytes});

    assign m_axis_cmd_tvalid = busy && (cmds_left != 16'd0);
    assign m_axis_cmd_tdata  = {4'h0, 4'h0, addr, 1'b0, 1'b1, 6'd0, 1'b1, btt};

    wire beat = s_axis_tvalid && s_axis_tready;
    assign s_axis_tready = busy && (beats_left != 23'd0);
    assign wbuf_we       = beat && !r_sel_a;
    assign abuf_we       = beat &&  r_sel_a;
    assign abuf_bank     = r_bank;
    assign buf_wdata     = s_axis_tdata;

    assign s_axis_sts_tready = 1'b1;

    always @(posedge clk) begin
        if (rst) begin
            busy       <= 1'b0;
            err        <= 1'b0;
            addr       <= 32'd0;
            step       <= 32'd0;
            btt        <= 23'd0;
            cmds_left  <= 16'd0;
            beats_left <= 23'd0;
            buf_waddr  <= 16'd0;
            r_sel_a    <= 1'b0;
            r_bank     <= 1'b0;
        end else begin
            if (start) begin
                busy       <= 1'b1;
                addr       <= base;
                step       <= contig ? 32'd0 : stride;
                btt        <= contig ? total[22:0] : {7'd0, row_bytes};
                cmds_left  <= contig ? 16'd1 : rows;
                beats_left <= total[22:0] >> BSH;
                buf_waddr  <= 16'd0;
                r_sel_a    <= sel_a;
                r_bank     <= bank;
            end else begin
                if (m_axis_cmd_tvalid && m_axis_cmd_tready) begin
                    addr      <= addr + step;
                    cmds_left <= cmds_left - 16'd1;
                end
                if (beat) begin
                    buf_waddr  <= buf_waddr + 16'd1;
                    beats_left <= beats_left - 23'd1;
                    if (beats_left == 23'd1)
                        busy <= 1'b0;
                end
            end

            if (job_start)
                err <= 1'b0;
            else if (s_axis_sts_tvalid && |s_axis_sts_tdata[6:4])
                err <= 1'b1;
        end
    end

endmodule
