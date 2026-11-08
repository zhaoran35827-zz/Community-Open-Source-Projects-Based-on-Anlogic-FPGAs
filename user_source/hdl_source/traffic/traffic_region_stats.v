`timescale 1ns/1ps
// Queue populations, never accumulated track IDs. Six boxes share one geometry
// datapath. CCL outputs must stay stable until the next I_result_valid strobe.
// Input is four adjacent pixels, mask[3] at I_x. The middle ROI is group-aligned.
module traffic_region_stats #(
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720,TARGETS=6,
    parameter A_X0=64,A_X1=479,C_X0=480,C_X1=799,B_X0=800,B_X1=1215,
    parameter ROI_Y0=160,ROI_Y1=639,
    parameter STABLE_FRAMES=3,EMPTY_FRAMES=6,CENTER_PIXELS=16
)(
    input I_clk,I_rst_n,I_valid,I_start,I_mode_good,
    input [15:0] I_x,I_y,
    input [3:0] I_mask,
    input I_result_valid,I_limit,
    input [4:0] I_reason,
    input [2:0] I_count,
    input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
    output reg O_update,O_good,O_occupied,
    output reg [2:0] O_count_a,O_count_b
);
    localparam PW=(CENTER_PIXELS<2)?1:$clog2(CENTER_PIXELS+4);
    localparam SW=(STABLE_FRAMES<2)?1:$clog2(STABLE_FRAMES+1);
    localparam EW=(EMPTY_FRAMES<2)?1:$clog2(EMPTY_FRAMES+1);
    localparam IW=(TARGETS<2)?1:$clog2(TARGETS);
    reg [PW-1:0] center_pixels;
    reg center_frame_busy,frame_complete,frame_pending,frame_bad,mode_d;
    reg [SW-1:0] good_run,run_a,run_b;
    reg [EW-1:0] empty_run;
    reg [2:0] candidate_a,candidate_b,scan_a,scan_b,scan_count;
    reg scanning,scan_bad,scan_busy;
    reg [IW-1:0] scan_index;

    wire in_center=I_x>=C_X0 && I_x<=C_X1-3 && I_y>=ROI_Y0 && I_y<=ROI_Y1;
    wire [2:0] hits=in_center ? ({2'b0,I_mask[3]}+{2'b0,I_mask[2]}+
                               {2'b0,I_mask[1]}+{2'b0,I_mask[0]}) : 3'd0;
    wire [PW:0] pixel_sum={1'b0,(I_start?{PW{1'b0}}:center_pixels)}+hits;
    wire pixel_busy=I_valid && pixel_sum>=CENTER_PIXELS;
    // A mux and a single pair of midpoint adders are shared across all targets.
    wire [15:0] x0=I_xmin[scan_index*16+:16],x1=I_xmax[scan_index*16+:16];
    wire [15:0] y0=I_ymin[scan_index*16+:16],y1=I_ymax[scan_index*16+:16];
    wire [16:0] sx={1'b0,x0}+{1'b0,x1},sy={1'b0,y0}+{1'b0,y1};
    wire box_valid=x0<=x1 && y0<=y1 && x1<IMG_WIDTH && y1<IMG_HEIGHT;
    wire box_in_y=sy>=2*ROI_Y0 && sy<2*(ROI_Y1+1);
    wire box_a=box_valid && box_in_y && sx>=2*A_X0 && sx<2*(A_X1+1);
    wire box_b=box_valid && box_in_y && sx>=2*B_X0 && sx<2*(B_X1+1);
    wire box_center=box_valid && x0<=C_X1 && x1>=C_X0 && y0<=ROI_Y1 && y1>=ROI_Y0;
    wire have_box=scan_index<scan_count;
    wire [2:0] final_a=scan_a+(have_box && box_a);
    wire [2:0] final_b=scan_b+(have_box && box_b);
    wire final_bad=scan_bad || (have_box && !box_valid);
    wire final_busy=scan_busy || (have_box && box_center) || pixel_busy;
    wire finish_scan=scan_count==0 || scan_index==scan_count-1'b1;
    // A second start without a consumed result means the preceding frame was
    // dropped; a result without a completed frame (or while scanning) is invalid.
    wire protocol_bad=(I_start && (frame_pending || scanning)) ||
        (I_result_valid && (!frame_complete || !frame_pending || frame_bad ||
                           I_start || scanning || I_limit || I_reason!=0 || I_count>TARGETS));

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            center_pixels<=0;center_frame_busy<=0;frame_complete<=0;frame_pending<=0;frame_bad<=1;mode_d<=0;
            O_update<=0;O_good<=0;O_occupied<=1;O_count_a<=0;O_count_b<=0;
            empty_run<=0;good_run<=0;run_a<=0;run_b<=0;candidate_a<=0;candidate_b<=0;
            scanning<=0;scan_bad<=0;scan_busy<=0;scan_index<=0;scan_count<=0;scan_a<=0;scan_b<=0;
        end else begin
            O_update<=0;mode_d<=I_mode_good;
            if(I_start) begin
                center_pixels<=0;center_frame_busy<=0;frame_complete<=0;
                frame_pending<=1;frame_bad<=!I_mode_good;
            end
            if(I_valid) begin
                if(I_start || center_pixels<CENTER_PIXELS) center_pixels<=pixel_sum[PW-1:0];
                if(pixel_busy) begin
                    center_frame_busy<=1;O_occupied<=1;empty_run<=0;
                    if(!O_occupied) O_update<=1;
                end
                if(I_x==IMG_WIDTH-4 && I_y==IMG_HEIGHT-1 && frame_pending) frame_complete<=1;
            end
            if(I_result_valid) begin
                frame_complete<=0;frame_pending<=0;
                if(!protocol_bad && I_mode_good) begin
                    scanning<=1;scan_index<=0;scan_count<=I_count;
                    scan_a<=0;scan_b<=0;scan_bad<=0;scan_busy<=center_frame_busy;
                end
            end
            if(scanning) begin
                scan_a<=final_a;scan_b<=final_b;scan_bad<=final_bad;scan_busy<=final_busy;
                if(have_box && box_center) begin
                    O_occupied<=1;empty_run<=0;
                    if(!O_occupied) O_update<=1;
                end
                if(finish_scan) begin
                    scanning<=0;O_update<=1;
                    if(final_bad) begin
                        O_good<=0;O_occupied<=1;empty_run<=0;good_run<=0;run_a<=0;run_b<=0;
                    end else begin
                        if(good_run<STABLE_FRAMES) good_run<=good_run+1'b1;
                        if(STABLE_FRAMES<=1 || good_run>=STABLE_FRAMES-1) O_good<=1;
                        if(final_a!=candidate_a) begin
                            candidate_a<=final_a;run_a<=1;
                            if(STABLE_FRAMES<=1) O_count_a<=final_a;
                        end else begin
                            if(run_a<STABLE_FRAMES) run_a<=run_a+1'b1;
                            if(STABLE_FRAMES<=1 || run_a>=STABLE_FRAMES-1) O_count_a<=final_a;
                        end
                        if(final_b!=candidate_b) begin
                            candidate_b<=final_b;run_b<=1;
                            if(STABLE_FRAMES<=1) O_count_b<=final_b;
                        end else begin
                            if(run_b<STABLE_FRAMES) run_b<=run_b+1'b1;
                            if(STABLE_FRAMES<=1 || run_b>=STABLE_FRAMES-1) O_count_b<=final_b;
                        end
                        if(final_busy) begin O_occupied<=1;empty_run<=0;end
                        else begin
                            if(empty_run<EMPTY_FRAMES) empty_run<=empty_run+1'b1;
                            if(EMPTY_FRAMES<=1 || empty_run>=EMPTY_FRAMES-1) O_occupied<=0;
                        end
                    end
                end else scan_index<=scan_index+1'b1;
            end
            // Highest priority: invalidate even when no CCL result is emitted.
            if(!I_mode_good || protocol_bad) begin
                O_good<=0;O_occupied<=1;empty_run<=0;good_run<=0;run_a<=0;run_b<=0;scanning<=0;
                if(!I_mode_good) frame_bad<=1;
                if((mode_d && !I_mode_good) || protocol_bad) O_update<=1;
            end
        end
    end
endmodule
