`timescale 1ns/1ps
// All four asynchronous contacts are accepted as one stable vector.
// SW1 cartoon, SW2 target, SW3 motion, SW4 edge split; up=1.
// Two synchronizer stages feed a shared debounce counter in the video domain.
// Actual feature activation is subsequently committed by frame_feature_flags.
module switch_feature_control #(parameter STABLE_CYCLES=1000000)(
    input I_clk,I_rst_n,input [3:0] I_switch,
    output [2:0] O_requested,output reg [3:0] O_stable
);
    localparam CW=(STABLE_CYCLES<2)?1:$clog2(STABLE_CYCLES);
    (* async_reg="true" *) reg [3:0] sync_meta,sync_value;
    reg [3:0] candidate;
    reg [CW-1:0] stable_count;
    assign O_requested=O_stable[2:0];
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            sync_meta<=0;sync_value<=0;candidate<=0;stable_count<=0;O_stable<=0;
        end else begin
            sync_meta<=I_switch;sync_value<=sync_meta;
            if(sync_value!=candidate)begin
                candidate<=sync_value;
                stable_count<=0;
            end else if(sync_value!=O_stable)begin
                if(STABLE_CYCLES<=1 || stable_count==STABLE_CYCLES-1)begin
                    O_stable<=candidate;stable_count<=0;
                end else stable_count<=stable_count+1'b1;
            end else stable_count<=0;
        end
    end
endmodule
