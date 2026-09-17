`timescale 1ns / 1ps

module tpu_accelerator_top #(
    parameter N = 4,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32,
    parameter ADDR_WIDTH = 10
)(
    input  wire clk,
    input  wire reset,

    input  wire start,
    output wire done,

    input  wire ps_ena_act,
    input  wire ps_wea_act,
    input  wire [ADDR_WIDTH-1:0] ps_addr_act,
    input  wire [(N*DATA_WIDTH)-1:0] ps_din_act,

    input  wire ps_ena_wt,
    input  wire ps_wea_wt,
    input  wire [ADDR_WIDTH-1:0] ps_addr_wt,
    input  wire [(N*DATA_WIDTH)-1:0] ps_din_wt,

    input  wire ps_ena_out,
    input  wire [ADDR_WIDTH-1:0] ps_addr_out,
    output wire [(N*ACC_WIDTH)-1:0] ps_dout_out
);

    wire fsm_en_act, fsm_en_wt, fsm_en_out, fsm_we_out;
    wire [ADDR_WIDTH-1:0] fsm_addr_act, fsm_addr_wt, fsm_addr_out;
    wire load_weight, core_valid;

    wire [(N*DATA_WIDTH)-1:0] act_data_to_core;
    wire [(N*DATA_WIDTH)-1:0] wt_data_to_core;
    wire [(N*ACC_WIDTH)-1:0]  psum_data_from_core;

    tpu_fsm #(.N(N), .ADDR_WIDTH(ADDR_WIDTH)) controller (
        .clk(clk), .reset(reset), .start(start), .done(done),
        .en_act(fsm_en_act),   .addr_act(fsm_addr_act),
        .en_wt(fsm_en_wt),     .addr_wt(fsm_addr_wt),
        .en_out(fsm_en_out),   .addr_out(fsm_addr_out), .we_out(fsm_we_out),
        .load_weight(load_weight), .core_valid(core_valid)
    );

    dual_port_bram #(.DATA_WIDTH(N*DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) act_bram (
        .clk(clk),
        .ena(ps_ena_act), .wea(ps_wea_act), .addra(ps_addr_act), .dina(ps_din_act),                .douta(),
        .enb(fsm_en_act), .web(1'b0),       .addrb(fsm_addr_act), .dinb({(N*DATA_WIDTH){1'b0}}),    .doutb(act_data_to_core)
    );

    dual_port_bram #(.DATA_WIDTH(N*DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) weight_bram (
        .clk(clk),
        .ena(ps_ena_wt), .wea(ps_wea_wt), .addra(ps_addr_wt), .dina(ps_din_wt),                .douta(),
        .enb(fsm_en_wt), .web(1'b0),      .addrb(fsm_addr_wt), .dinb({(N*DATA_WIDTH){1'b0}}),   .doutb(wt_data_to_core)
    );

    dual_port_bram #(.DATA_WIDTH(N*ACC_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) out_bram (
        .clk(clk),
        .ena(ps_ena_out), .wea(1'b0),       .addra(ps_addr_out),  .dina({(N*ACC_WIDTH){1'b0}}),    .douta(ps_dout_out),
        .enb(fsm_en_out), .web(fsm_we_out), .addrb(fsm_addr_out), .dinb(psum_data_from_core),      .doutb()
    );

    compute_core #(.N(N), .DATA_WIDTH(DATA_WIDTH), .ACC_WIDTH(ACC_WIDTH)) datapath (
        .clk(clk), .reset(reset),
        .load_weight(load_weight), .valid(core_valid),
        .act_in(act_data_to_core),
        .weight_in(wt_data_to_core),
        .psum_out(psum_data_from_core)
    );

endmodule