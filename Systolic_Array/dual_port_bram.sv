`timescale 1ns / 1ps

module dual_port_bram #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 10 
)(
    input  wire clk,

    input  wire ena,
    input  wire wea,
    input  wire [ADDR_WIDTH-1:0] addra,
    input  wire [DATA_WIDTH-1:0] dina,
    output reg  [DATA_WIDTH-1:0] douta,

    input  wire enb,
    input  wire web,
    input  wire [ADDR_WIDTH-1:0] addrb,
    input  wire [DATA_WIDTH-1:0] dinb,
    output reg  [DATA_WIDTH-1:0] doutb
);

    localparam ADDR_SHIFT = $clog2(DATA_WIDTH / 8);

    reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

    always @(posedge clk) begin
        if (ena) begin
            if (wea)
                ram[addra >> ADDR_SHIFT] <= dina;
            douta <= ram[addra >> ADDR_SHIFT];
        end
    end

    always @(posedge clk) begin
        if (enb) begin
            if (web)
                ram[addrb] <= dinb;
            doutb <= ram[addrb];
        end
    end

endmodule