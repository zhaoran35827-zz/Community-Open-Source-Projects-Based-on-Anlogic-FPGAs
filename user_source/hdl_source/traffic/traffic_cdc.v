`timescale 1ns/1ps
// Bundled-data mailbox. Payload is held until the receiving clock acknowledges
// its request toggle. When busy, keep the most recent unsent snapshot.
module traffic_cdc #(parameter WIDTH=8)(
    input I_src_clk,I_src_rst_n,I_send,
    input [WIDTH-1:0] I_data,
    input I_dst_clk,I_dst_rst_n,
    output reg O_update,
    output reg [WIDTH-1:0] O_data
);
    reg [WIDTH-1:0] held,pending_data;
    reg request,acknowledge,pending;
    (* async_reg="true" *) reg ack_meta,ack_sync,req_meta,req_sync;
    (* async_reg="true", keep="true" *) reg [WIDTH-1:0] payload_meta,payload_sync;
    reg settling;
    always @(posedge I_src_clk or negedge I_src_rst_n) begin
        if(!I_src_rst_n) begin
            held<=0;pending_data<=0;request<=0;pending<=0;
            ack_meta<=0;ack_sync<=0;
        end else begin
            ack_meta<=acknowledge;ack_sync<=ack_meta;
            if(request==ack_sync) begin
                if(I_send || pending) begin
                    held<=I_send?I_data:pending_data;
                    request<=~request;pending<=0;
                end
            end else if(I_send) begin pending_data<=I_data;pending<=1;end
        end
    end
    always @(posedge I_dst_clk or negedge I_dst_rst_n) begin
        if(!I_dst_rst_n) begin
            req_meta<=0;req_sync<=0;acknowledge<=0;O_data<=0;O_update<=0;
            payload_meta<=0;payload_sync<=0;settling<=0;
        end else begin
            req_meta<=request;req_sync<=req_meta;O_update<=0;
            payload_meta<=held;payload_sync<=payload_meta;
            // An extra settling cycle after the synchronized request: payload
            // may miss the first sampling edge, and still arrives coherently.
            if(settling)begin O_data<=payload_sync;O_update<=1;acknowledge<=req_sync;settling<=0;end
            else if(req_sync!=acknowledge)settling<=1;
        end
    end
endmodule

// External SPST switch: J2-7/F14 pulled up; closing to GND enables traffic.
// This switch is independent of the four image-algorithm switches.
module traffic_enable_control #(parameter STABLE_CYCLES=1000000)(
    input I_clk,I_rst_n,I_switch_n,
    output reg O_enable
);
    localparam CW=(STABLE_CYCLES<2)?1:$clog2(STABLE_CYCLES);
    (* async_reg="true" *) reg sync_meta,sync_value;
    reg candidate;
    reg [CW-1:0] stable_count;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            sync_meta<=1;sync_value<=1;candidate<=1;stable_count<=0;O_enable<=0;
        end else begin
            sync_meta<=I_switch_n;sync_value<=sync_meta;
            if(sync_value!=candidate)begin
                candidate<=sync_value;stable_count<=0;
            end else if(O_enable==candidate)begin
                if(STABLE_CYCLES<=1 || stable_count==STABLE_CYCLES-1)begin
                    O_enable<=~candidate;stable_count<=0;
                end else stable_count<=stable_count+1'b1;
            end else stable_count<=0;
        end
    end
endmodule
