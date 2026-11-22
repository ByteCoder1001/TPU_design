`timescale 1ns / 1ps

module compute_core #(
    parameter N = 4,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire clk,
    input  wire reset,
    input  wire load_weight,
    input  wire valid,

    input  wire [N-1:0][DATA_WIDTH-1:0] act_in,
    input  wire [N-1:0][DATA_WIDTH-1:0] weight_in,

    output wire [N-1:0][ACC_WIDTH-1:0]  psum_out
);

    wire [N-1:0][DATA_WIDTH-1:0] skewed_act;
    wire [N-1:0][ACC_WIDTH-1:0]  array_psum_out;

    wire [N-1:0][ACC_WIDTH-1:0] psum_top_mux;

    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : wt_mux
            assign psum_top_mux[g] = load_weight ?
                {{(ACC_WIDTH-DATA_WIDTH){1'b0}}, weight_in[g]} : {ACC_WIDTH{1'b0}};
        end
    endgenerate

    skew_buffer #(
        .N(N),
        .DATA_WIDTH(DATA_WIDTH)
    ) skew_inst (
        .clk(clk),
        .en(valid),
        .data_in(act_in),
        .data_out(skewed_act)
    );

    systolic_array_core #(
        .N(N),
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH(ACC_WIDTH)
    ) array_inst (
        .clk(clk),
        .reset(reset),
        .load_weight(load_weight),
        .valid(valid),
        .act_in_left(skewed_act),
        .psum_in_top(psum_top_mux),
        .psum_out_bottom(array_psum_out)
    );

    deskew_buffer #(
        .N(N),
        .DATA_WIDTH(ACC_WIDTH)
    ) deskew_inst (
        .clk(clk),
        .en(valid),
        .data_in(array_psum_out),
        .data_out(psum_out)
    );

endmodule