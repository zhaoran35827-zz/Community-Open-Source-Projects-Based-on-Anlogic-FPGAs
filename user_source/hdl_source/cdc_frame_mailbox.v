// Bundled-data, two-phase handshake. Both domains must share reset assertion.
// The source holds the payload until the destination has sampled it and acked.
// A frame event must be spaced far enough apart for O_src_ready to return high.
module cdc_frame_mailbox #(
    parameter integer WIDTH = 16
)(
    input  wire             I_src_clk,
    input  wire             I_src_rst_n,
    input  wire             I_src_valid,
    input  wire [WIDTH-1:0]  I_src_data,
    output wire             O_src_ready,
    input  wire             I_dst_clk,
    input  wire             I_dst_rst_n,
    output reg              O_dst_valid,
    output reg  [WIDTH-1:0]  O_dst_data
);
    reg [WIDTH-1:0] S_payload_hold;
    reg S_request;
    reg S_ack;
    (* ASYNC_REG = "TRUE" *) reg S_ack_meta, S_ack_sync;
    (* ASYNC_REG = "TRUE" *) reg S_request_meta, S_request_sync;
    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] S_payload_meta, S_payload_sync;
    reg S_pending;
    (* ASYNC_REG = "TRUE" *) reg [1:0] S_src_reset_sync, S_dst_reset_sync;
    wire S_src_rst_n = S_src_reset_sync[1];
    wire S_dst_rst_n = S_dst_reset_sync[1];

    // Asynchronous assertion, synchronous deassertion in each local domain.
    always @(posedge I_src_clk or negedge I_src_rst_n) begin
        if(!I_src_rst_n)
            S_src_reset_sync <= 2'b00;
        else
            S_src_reset_sync <= {S_src_reset_sync[0],1'b1};
    end
    always @(posedge I_dst_clk or negedge I_dst_rst_n) begin
        if(!I_dst_rst_n)
            S_dst_reset_sync <= 2'b00;
        else
            S_dst_reset_sync <= {S_dst_reset_sync[0],1'b1};
    end

    assign O_src_ready = S_src_rst_n && (S_request == S_ack_sync);

    always @(posedge I_src_clk or negedge S_src_rst_n) begin
        if(!S_src_rst_n) begin
            S_payload_hold <= {WIDTH{1'b0}};
            S_request <= 1'b0;
            S_ack_meta <= 1'b0;
            S_ack_sync <= 1'b0;
        end
        else begin
            S_ack_meta <= S_ack;
            S_ack_sync <= S_ack_meta;
            if(I_src_valid && O_src_ready) begin
                S_payload_hold <= I_src_data;
                S_request <= ~S_request;
            end
        end
    end

    always @(posedge I_dst_clk or negedge S_dst_rst_n) begin
        if(!S_dst_rst_n) begin
            S_request_meta <= 1'b0;
            S_request_sync <= 1'b0;
            S_payload_meta <= {WIDTH{1'b0}};
            S_payload_sync <= {WIDTH{1'b0}};
            S_pending <= 1'b0;
            S_ack <= 1'b0;
            O_dst_valid <= 1'b0;
            O_dst_data <= {WIDTH{1'b0}};
        end
        else begin
            S_request_meta <= S_request;
            S_request_sync <= S_request_meta;
            S_payload_meta <= S_payload_hold;
            S_payload_sync <= S_payload_meta;
            O_dst_valid <= 1'b0;
            if(S_pending) begin
                // One extra settling cycle after the synchronized request.
                O_dst_data <= S_payload_sync;
                O_dst_valid <= 1'b1;
                S_ack <= S_request_sync;
                S_pending <= 1'b0;
            end
            else if(S_request_sync != S_ack)
                S_pending <= 1'b1;
        end
    end
endmodule
