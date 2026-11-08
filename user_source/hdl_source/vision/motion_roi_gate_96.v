`timescale 1ns/1ps
// Previous motion proposal PLUS all known target boxes. Refresh full-frame
// regularly, on warmup, on no proposal/track, and following a CCL limit.
// Gate only the mask; NEVER gate video valid or the full-frame coordinates.
module motion_roi_gate_96 #(
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720,REFRESH_PERIOD=8,
    parameter MOTION_PAD=128,TRACK_PAD=128, TARGETS=4, CB=(TARGETS<8?3:4)
)(
    input I_clk,I_rst_n,I_valid,I_start,I_motion_enable,I_motion_ready,
    input I_motion_box_valid,
    input [15:0] I_motion_xmin,I_motion_xmax,I_motion_ymin,I_motion_ymax,
    input [CB-1:0] I_count,
    input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
    input [4:0] I_reason,
    input [15:0] I_x,I_y,
    input [3:0] I_mask,
    output [3:0] O_mask,
    output O_full_scan
);
    reg [15:0] frame_index;
    reg full_scan;
    reg [TARGETS:0] box_valid;
    reg [(TARGETS+1)*16-1:0] xmin,xmax,ymin,ymax;
    wire requested_full=!I_motion_enable || !I_motion_ready ||
        (frame_index%REFRESH_PERIOD==0) || I_reason!=0 ||
        (!I_motion_box_valid && I_count==0);
    assign O_full_scan=I_start?requested_full:full_scan;
    function [15:0] lower;
        input [15:0] x;input integer pad;
        begin lower=(x<pad)?0:x-pad;end
    endfunction
    function [15:0] upper;
        input [15:0] x;input integer pad,limit;
        begin upper=({1'b0,x}+pad>=limit)?limit-1:x+pad;end
    endfunction
    integer t;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin frame_index<=0;full_scan<=1;box_valid<=0;xmin<=0;xmax<=0;ymin<=0;ymax<=0;end
        else if(I_start)begin
            frame_index<=frame_index+1'b1;full_scan<=requested_full;
            box_valid[TARGETS]<=I_motion_box_valid;
            xmin[TARGETS*16 +:16]<=lower(I_motion_xmin,MOTION_PAD);
            xmax[TARGETS*16 +:16]<=upper(I_motion_xmax,MOTION_PAD,IMG_WIDTH);
            ymin[TARGETS*16 +:16]<=lower(I_motion_ymin,MOTION_PAD);
            ymax[TARGETS*16 +:16]<=upper(I_motion_ymax,MOTION_PAD,IMG_HEIGHT);
            for(t=0;t<TARGETS;t=t+1)begin
                box_valid[t]<=t<I_count;
                xmin[t*16 +:16]<=lower(I_xmin[t*16 +:16],TRACK_PAD);
                xmax[t*16 +:16]<=upper(I_xmax[t*16 +:16],TRACK_PAD,IMG_WIDTH);
                ymin[t*16 +:16]<=lower(I_ymin[t*16 +:16],TRACK_PAD);
                ymax[t*16 +:16]<=upper(I_ymax[t*16 +:16],TRACK_PAD,IMG_HEIGHT);
            end
        end
    end
    reg included;
    integer b;
    always @*begin
        // Conservative beat-level membership avoids four comparator copies.
        // Boundaries may include up to three extra pixels, never drop a lane.
        included=O_full_scan || I_start;
        for(b=0;b<TARGETS+1;b=b+1)
            if(box_valid[b] && I_x[15:2]>=xmin[b*16+2 +:14] &&
                I_x[15:2]<=xmax[b*16+2 +:14] &&
                I_y>=ymin[b*16 +:16] && I_y<=ymax[b*16 +:16])included=1;
    end
    assign O_mask=(I_valid && included)?I_mask:4'd0;
endmodule
