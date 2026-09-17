`timescale 1ns / 1ps

module tpu_fsm #(
    parameter N = 4,
    parameter ADDR_WIDTH = 10
)(
    input  wire clk,
    input  wire reset,
    input  wire start,
    output reg  done,

    output reg  en_act,
    output reg  [ADDR_WIDTH-1:0] addr_act,

    output reg  en_wt,
    output reg  [ADDR_WIDTH-1:0] addr_wt,

    output wire en_out,
    output wire we_out,
    output reg  [ADDR_WIDTH-1:0] addr_out,

    output wire load_weight,
    output wire core_valid
);

    localparam [1:0] IDLE    = 2'd0;
    localparam [1:0] LOAD_WT = 2'd1;
    localparam [1:0] COMPUTE = 2'd2;
    localparam [1:0] DONE_ST = 2'd3;

    reg [1:0] state;
    reg [7:0] counter;

    reg load_weight_raw, core_valid_raw;
    reg load_weight_pipe, core_valid_pipe;

    assign load_weight = load_weight_pipe;
    assign core_valid  = core_valid_pipe;

    always @(posedge clk) begin
        if (reset) begin
            load_weight_pipe <= 1'b0;
            core_valid_pipe  <= 1'b0;
        end else begin
            load_weight_pipe <= load_weight_raw;
            core_valid_pipe  <= core_valid_raw;
        end
    end

    localparam PIPELINE_DEPTH = 2 * N;

    reg [PIPELINE_DEPTH-1:0] we_delay_pipe;

    always @(posedge clk) begin
        if (reset) begin
            state           <= IDLE;
            counter         <= 8'd0;
            addr_wt         <= {ADDR_WIDTH{1'b0}};
            addr_act        <= {ADDR_WIDTH{1'b0}};
            addr_out        <= {ADDR_WIDTH{1'b0}};
            en_wt           <= 1'b0;
            en_act          <= 1'b0;
            load_weight_raw <= 1'b0;
            core_valid_raw  <= 1'b0;
            we_delay_pipe   <= {PIPELINE_DEPTH{1'b0}};
            done            <= 1'b0;
        end else begin
 
            we_delay_pipe <= {we_delay_pipe[PIPELINE_DEPTH-2:0], en_act};

            if (we_out) begin
                addr_out <= addr_out + 1'b1;
            end

            case (state)
                IDLE: begin
                    done     <= 1'b0;
                    addr_wt  <= {ADDR_WIDTH{1'b0}};
                    addr_act <= {ADDR_WIDTH{1'b0}};
                    addr_out <= {ADDR_WIDTH{1'b0}};
                    if (start) begin
                        state           <= LOAD_WT;
                        en_wt           <= 1'b1;
                        addr_wt         <= N - 1;         
                        load_weight_raw <= 1'b1;
                        counter         <= 8'd0;
                    end
                end

                LOAD_WT: begin
                    if (counter < N - 1) begin
                        counter <= counter + 1'b1;
                        addr_wt <= addr_wt - 1'b1;
                    end else begin
                        en_wt           <= 1'b0;
                        load_weight_raw <= 1'b0;

                        state          <= COMPUTE;
                        en_act         <= 1'b1;
                        core_valid_raw <= 1'b1;
                        counter        <= 8'd0;
                    end
                end

                COMPUTE: begin
                    if (counter < N - 1) begin
                        counter  <= counter + 1'b1;
                        addr_act <= addr_act + 1'b1;
                    end else if (counter == N - 1) begin
                        en_act  <= 1'b0;
                        counter <= counter + 1'b1;
                    end else if (counter < (N + PIPELINE_DEPTH)) begin
                        counter <= counter + 1'b1;
                    end else begin
                        core_valid_raw <= 1'b0;
                        state          <= DONE_ST;
                    end
                end

                DONE_ST: begin
                    done <= 1'b1;
                    if (!start) begin
                        state <= IDLE;
                    end
                end
            endcase
        end
    end

    assign we_out = we_delay_pipe[PIPELINE_DEPTH-1];
    assign en_out = we_out;

endmodule