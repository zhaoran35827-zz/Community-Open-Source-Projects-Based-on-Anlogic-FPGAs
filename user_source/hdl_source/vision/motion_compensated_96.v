`timescale 1ns/1ps
// Regionally compensated frame difference: ping-pong references, no scene-wide veto.
// Store one compact RGB332 signature per 4x4 tile, not full RGB frames.
// Component-wise differences avoid losing equal-luminance colored targets.
// Immutable ping-pong references; two-frame warmup saturates (no wrap).
// All video, red-mask and flag paths have exactly TWO register stages.
module motion_compensated_96 #(
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720,
    parameter SS_X=1,SS_Y=4,FRAME_INIT_SKIP=2,MIN_SAMPLES=4,
    parameter COLOR_DELTA=1, NOISE_GUARD=8,
    parameter SUPPORT_MOTION=1,
    parameter [2:0] DEFAULT_FLAGS=3'b111,
    parameter FRAME_PERIOD=1, SAMPLE_FILTER=0,CAMERA_DETAIL=0
)(
    input I_clk,I_rst_n,I_valid,I_start,I_last,
    input [95:0] I_data,I_detect_rgb,
    input [3:0] I_red_mask,
    input [2:0] I_flags, // motion,target,cartoon
    output reg O_valid,O_start,O_last,
    output reg [95:0] O_data,
    output reg [3:0] O_red_mask,
    output reg [2:0] O_flags,
    output reg O_motion_mask,
    output O_ready,
    output reg O_bbox_valid,
    output reg [15:0] O_xmin,O_xmax,O_ymin,O_ymax,
    output reg [15:0] O_sample_count,output [23:0] O_camera_vectors
);
    localparam BEATS=IMG_WIDTH/4,COLS=BEATS/SS_X,ROWS=IMG_HEIGHT/SS_Y;
    localparam DEPTH=COLS*ROWS,AB=(DEPTH<2)?1:$clog2(DEPTH);
    localparam CB=(COLS<2)?1:$clog2(COLS),FW=(FRAME_INIT_SKIP<1)?1:$clog2(FRAME_INIT_SKIP+1);
    reg [$clog2(BEATS)-1:0] beat;
    reg [$clog2(IMG_HEIGHT)-1:0] row;
    reg [FW-1:0] warm_frames;
    localparam PW=(FRAME_PERIOD<2)?1:$clog2(FRAME_PERIOD);
    reg [PW-1:0] cadence;
    wire compare_frame=FRAME_PERIOD<=1 || warm_frames<FRAME_INIT_SKIP || cadence==0;
    wire [15:0] px=I_start?16'd0:beat,py=I_start?16'd0:row;
    wire keep=(px%SS_X==0)&&(py%SS_Y==0);
    wire eof=I_valid && px==BEATS-1 && py==IMG_HEIGHT-1;
    wire enabled=SUPPORT_MOTION!=0 && I_flags[2];
    // Filter the detection tap only. Video/CCL retain every original pixel.
    // Four-pixel box average attenuates single-sensel impulses before RGB332.
    function [23:0] mean4;input [95:0] pixels;
    integer c;reg [9:0] sum;
    begin for(c=0;c<3;c=c+1)begin
      sum={2'b0,pixels[c*8+:8]}+{2'b0,pixels[24+c*8+:8]}+
          {2'b0,pixels[48+c*8+:8]}+{2'b0,pixels[72+c*8+:8]}+10'd2;
      mean4[c*8+:8]=sum[9:2];end end endfunction
    wire [23:0] first_pixel=SAMPLE_FILTER!=0?mean4(I_detect_rgb):I_detect_rgb[95:72];
    wire [7:0] signature={first_pixel[23:21],first_pixel[15:13],first_pixel[7:6]};
    wire [7:0] old_signature;
    wire sample_event=I_rst_n && I_valid && keep && enabled;
    wire ram_event=sample_event && compare_frame;
    assign O_ready=enabled && warm_frames>=FRAME_INIT_SKIP;
    wire [23:0] camera_vectors;
    reg [23:0] row_camera_vectors;
    wire [23:0] applied_vectors=(keep && px==0)?camera_vectors:row_camera_vectors;
    wire [1:0] quadrant={py>=IMG_HEIGHT/2,(px*4>=IMG_WIDTH/2)};
    wire [5:0] regional_vector=applied_vectors[quadrant*6+:6];
    wire [9:0] sample_col=px/SS_X,sample_row=py/SS_Y;
    wire reference_in_bounds;
    assign O_camera_vectors=camera_vectors;
    generate if(CAMERA_DETAIL!=0 && IMG_WIDTH>=512 && IMG_HEIGHT>=256 && IMG_WIDTH<=640 && IMG_HEIGHT<=360)begin:g_fine_camera
    camera_shift_fine_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_camera (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(enabled),.I_valid(I_valid),.I_start(I_start),
        .I_beat(px[8:0]),.I_y(py[10:0]),.I_signature(signature),
        .O_vectors(),.O_frame_vectors(camera_vectors));
    end else begin:g_legacy_camera
    camera_shift_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_camera (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(enabled),.I_valid(I_valid),.I_start(I_start),
        .I_beat(px[8:0]),.I_y(py[10:0]),.I_signature(signature),
        .O_vectors(),.O_frame_vectors(camera_vectors));
    end endgenerate
    packed_motion_pair #(.COLS(COLS),.ROWS(ROWS)) memory (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(I_start),.I_event(ram_event),
        .I_signature(signature),.I_col(sample_col),.I_row(sample_row),
        .I_dx(regional_vector[2:0]),.I_dy(regional_vector[5:3]),
        .O_previous(old_signature),.O_in_bounds(reference_in_bounds));
    reg v1,s1,l1,k1,ready1,e1,compare1;
    reg [95:0] data1;
    reg [3:0] mask1;
    reg [2:0] flags1;
    reg [7:0] sig1;
    reg [4:0] rfrac1,gfrac1;reg [5:0] bfrac1;
    reg [15:0] x1,y1;
    reg [CB-1:0] col1;
    reg row_keep1;
    reg motion_line[0:COLS-1]; // initialized by two full warmup frames
    wire [2:0] dr=(sig1[7:5]>=old_signature[7:5])?sig1[7:5]-old_signature[7:5]:old_signature[7:5]-sig1[7:5];
    wire [2:0] dg=(sig1[4:2]>=old_signature[4:2])?sig1[4:2]-old_signature[4:2]:old_signature[4:2]-sig1[4:2];
    wire [1:0] db=(sig1[1:0]>=old_signature[1:0])?sig1[1:0]-old_signature[1:0]:old_signature[1:0]-sig1[1:0];
    wire held_mask;
    // Hysteresis around the old RGB332 bin, not another gain/filter on video.
    wire r_changed=dr>=COLOR_DELTA && (dr>1 || (sig1[7:5]>old_signature[7:5]?rfrac1>=NOISE_GUARD:rfrac1<32-NOISE_GUARD));
    wire g_changed=dg>=COLOR_DELTA && (dg>1 || (sig1[4:2]>old_signature[4:2]?gfrac1>=NOISE_GUARD:gfrac1<32-NOISE_GUARD));
    wire b_changed=db>=1 && (db>1 || (sig1[1:0]>old_signature[1:0]?bfrac1>=NOISE_GUARD:bfrac1<64-NOISE_GUARD));
    wire moving=v1 && k1 && ready1 && compare1 && reference_in_bounds && (r_changed || g_changed || b_changed);
    wire supported_motion;
    generate if(SAMPLE_FILTER!=0)begin:g_sample_filter
        // Reject isolated 4x4 candidates BEFORE expansion. Reusing a causal
        // row and one left bit adds no video delay/full RGB line buffer.
        reg candidates[0:COLS-1];reg left_candidate;
        wire neighbor=(col1!=0 && left_candidate) || (y1>=SS_Y && candidates[col1]);
        assign supported_motion=moving && neighbor;
        always @(posedge I_clk or negedge I_rst_n)
          if(!I_rst_n)left_candidate<=0;
          else if(v1 && k1)begin candidates[col1]<=moving;left_candidate<=moving;end
    end else begin:g_unfiltered_samples
        assign supported_motion=moving;
    end endgenerate
    wire tile_motion=v1 && k1 && ready1 && (compare1?supported_motion:held_mask);
    generate if(SUPPORT_MOTION!=0 && FRAME_PERIOD>1)begin:g_mask_cache
        motion_mask_history #(.SAMPLES(DEPTH)) memory (
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(I_start),.I_event(sample_event),
            .I_update(v1 && k1 && flags1[2] && compare1),.I_bit(supported_motion),.O_previous(held_mask));
    end else begin:g_no_mask_cache
        assign held_mask=1'b0;
    end endgenerate
    wire [15:0] tile_x1=((x1+SS_X*4-1)<IMG_WIDTH)?x1+SS_X*4-1:IMG_WIDTH-1;
    wire [15:0] tile_y1=((y1+SS_Y-1)<IMG_HEIGHT)?y1+SS_Y-1:IMG_HEIGHT-1;
    reg [15:0] count_acc,xmin_acc,xmax_acc,ymin_acc,ymax_acc;
    wire [15:0] base_count=s1?16'd0:count_acc;
    wire [15:0] base_xmin=s1?IMG_WIDTH-1:xmin_acc,base_xmax=s1?16'd0:xmax_acc;
    wire [15:0] base_ymin=s1?IMG_HEIGHT-1:ymin_acc,base_ymax=s1?16'd0:ymax_acc;
    // Fine 4x2 sampling has 115200 tiles: 16-bit public sample count must
    // saturate, never wrap to zero and invalidate a large moving region.
    wire [15:0] next_count=base_count+((tile_motion && base_count!=16'hffff)?1'b1:1'b0);
    wire [15:0] next_xmin=(tile_motion && x1<base_xmin)?x1:base_xmin;
    wire [15:0] next_xmax=(tile_motion && tile_x1>base_xmax)?tile_x1:base_xmax;
    wire [15:0] next_ymin=(tile_motion && y1<base_ymin)?y1:base_ymin;
    wire [15:0] next_ymax=(tile_motion && tile_y1>base_ymax)?tile_y1:base_ymax;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            beat<=0;row<=0;warm_frames<=0;cadence<=0;compare1<=0;row_camera_vectors<=0;
            v1<=0;s1<=0;l1<=0;k1<=0;ready1<=0;e1<=0;data1<=0;mask1<=0;
            flags1<=DEFAULT_FLAGS;sig1<=0;rfrac1<=0;gfrac1<=0;bfrac1<=0;x1<=0;y1<=0;col1<=0;row_keep1<=0;
            O_valid<=0;O_start<=0;O_last<=0;O_data<=0;O_red_mask<=0;O_flags<=DEFAULT_FLAGS;O_motion_mask<=0;
            count_acc<=0;xmin_acc<=IMG_WIDTH-1;xmax_acc<=0;ymin_acc<=IMG_HEIGHT-1;ymax_acc<=0;
            O_bbox_valid<=0;O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;O_sample_count<=0;
        end else begin
            if(I_valid && keep && px==0)row_camera_vectors<=camera_vectors;
            if(!enabled)begin warm_frames<=0;cadence<=0;end
            else if(eof)begin
                if(warm_frames<FRAME_INIT_SKIP)begin warm_frames<=warm_frames+1'b1;cadence<=0;end
                else if(FRAME_PERIOD<=1 || cadence==FRAME_PERIOD-1)cadence<=0;
                else cadence<=cadence+1'b1;
            end
            if(I_start)begin beat<=0;row<=0;end
            if(I_valid)begin
                if(px==BEATS-1)begin beat<=0;row<=(py==IMG_HEIGHT-1)?0:py+1'b1;end
                else begin beat<=px+1'b1;row<=py;end
            end
            v1<=I_valid;s1<=I_start;l1<=I_last;k1<=keep;e1<=eof;
            compare1<=compare_frame;ready1<=O_ready;data1<=I_data;mask1<=I_red_mask;flags1<=I_flags;
            sig1<=signature;rfrac1<=first_pixel[20:16];gfrac1<=first_pixel[12:8];bfrac1<=first_pixel[5:0];x1<=px*4;y1<=py;col1<=px/SS_X;row_keep1<=py%SS_Y==0;
            O_valid<=v1;O_start<=s1;O_last<=l1;O_data<=data1;O_red_mask<=mask1;O_flags<=flags1;
            if(v1 && k1)motion_line[col1]<=tile_motion;
            O_motion_mask<=v1 && ready1 && flags1[2] &&
                ((row_keep1 && k1)?tile_motion:motion_line[col1]);
            if(s1 || v1)begin
                count_acc<=next_count;xmin_acc<=next_xmin;xmax_acc<=next_xmax;
                ymin_acc<=next_ymin;ymax_acc<=next_ymax;
            end
            if(e1)begin
                O_bbox_valid<=ready1 && flags1[2] && next_count>=MIN_SAMPLES;
                O_xmin<=next_xmin;O_xmax<=next_xmax;O_ymin<=next_ymin;O_ymax<=next_ymax;
                O_sample_count<=next_count;
                count_acc<=0;xmin_acc<=IMG_WIDTH-1;xmax_acc<=0;ymin_acc<=IMG_HEIGHT-1;ymax_acc<=0;
            end
            if(!flags1[2])O_bbox_valid<=0;
        end
    end
endmodule
