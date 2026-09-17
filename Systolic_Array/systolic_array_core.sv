`timescale 1ns / 1ps

module systolic_array_core #(
    parameter N = 4,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire clk,
    input  wire reset,
    input  wire load_weight,
    input  wire valid,

    input  wire [N-1:0][DATA_WIDTH-1:0] act_in_left,
    input  wire [N-1:0][ACC_WIDTH-1:0]  psum_in_top,

    output wire [N-1:0][ACC_WIDTH-1:0]  psum_out_bottom
);

    wire [DATA_WIDTH-1:0] act_wire  [0:N-1][0:N];
    wire [ACC_WIDTH-1:0]  psum_wire [0:N][0:N-1];

    genvar i, j;
    generate
        for (i = 0; i < N; i = i + 1) begin : edge_connections
            assign act_wire[i][0]     = act_in_left[i];
            assign psum_wire[0][i]    = psum_in_top[i];
            assign psum_out_bottom[i] = psum_wire[N][i];
        end

        for (i = 0; i < N; i = i + 1) begin : row
            for (j = 0; j < N; j = j + 1) begin : col
                tpu_pe #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ACC_WIDTH(ACC_WIDTH)
                ) pe_inst (
                    .clk        (clk),
                    .reset      (reset),
                    .load_weight(load_weight),
                    .valid      (valid),
                    .act_in     (act_wire[i][j]),
                    .psum_in    (psum_wire[i][j]),
                    .act_out    (act_wire[i][j+1]),
                    .psum_out   (psum_wire[i+1][j])
                );
            end
        end
    endgenerate
endmodule
