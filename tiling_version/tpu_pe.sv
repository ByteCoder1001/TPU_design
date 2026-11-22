`timescale 1ns / 1ps

module tpu_pe #(
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire clk,
    input  wire reset,
    input  wire load_weight,
    input  wire valid,

    input  wire [DATA_WIDTH-1:0] act_in,
    input  wire [ACC_WIDTH-1:0]  psum_in,

    output reg  [DATA_WIDTH-1:0] act_out,
    output reg  [ACC_WIDTH-1:0]  psum_out
);

    reg [DATA_WIDTH-1:0] weight_reg;

    always @(posedge clk) begin
        if (reset) begin
            weight_reg <= {DATA_WIDTH{1'b0}};
            act_out    <= {DATA_WIDTH{1'b0}};
            psum_out   <= {ACC_WIDTH{1'b0}};
        end else begin
            if (load_weight) begin
                weight_reg <= psum_in[DATA_WIDTH-1:0];  
                psum_out   <= psum_in;                   
            end else if (valid) begin
                psum_out <= psum_in + (act_in * weight_reg);
                act_out  <= act_in;
            end
        end
    end
endmodule
