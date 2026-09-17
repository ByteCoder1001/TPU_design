`timescale 1ns / 1ps

module skew_buffer #(
    parameter N = 4,
    parameter DATA_WIDTH = 8
)(
    input  wire clk,
    input  wire en,  
    
    input  wire [N-1:0][DATA_WIDTH-1:0] data_in,
    output wire [N-1:0][DATA_WIDTH-1:0] data_out
);

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : skew_row
            if (i == 0) begin : delay_0
                assign data_out[i] = data_in[i];
            end else begin : delay_n
                reg [DATA_WIDTH-1:0] delay_line [0:i-1];
                integer k;

                always @(posedge clk) begin
                    if (en) begin
                        delay_line[0] <= data_in[i];
                        for (k = 1; k < i; k = k + 1) begin
                            delay_line[k] <= delay_line[k-1];
                        end
                    end
                end

                assign data_out[i] = delay_line[i-1];
            end
        end
    endgenerate
endmodule
