`timescale 1ns/1ps
module fusion_vision_96 #(
    parameter EDGE_DISPLAY = 0,
    // Applies only to the color branch: 0 original, 1 cyan overlay, 2 mask.
    // Step 8 defaults to original color; mode 2 remains the mask build option.
    parameter COLOR_DEBUG_MODE = 0,
    // Use the aligned, pre-AWB RGB for color display on this resource-limited
    // device. 0 keeps the previous dynamic-AWB color build option.
    parameter DISPLAY_PRE_AWB = 1,
    // Display AWB stays dynamic; segmentation uses its aligned raw RGB tap.
    parameter AWB_FIXED_GAIN = 0,
    parameter DETECT_PRE_AWB = 1,
    parameter RGB_PROBE_ENABLE = 0,
    parameter BBOX_ENABLE = 1,
    // Step 5: independent red connected components; 0 restores step 4.
    parameter MULTI_TARGET_ENABLE = 1,
    parameter MASK_FILTER_ENABLE = 1,
    // Standard display: original color, cleaned-mask box and smoothing.
    parameter MASK_DIAGNOSTIC_ENABLE = 0,
    parameter BBOX_USE_RAW_MASK = 0,
    parameter BBOX_SMOOTH_ENABLE = 1,
    parameter BBOX_MIN_AREA = 100,
    parameter [7:0] RED_MIN = 8'd80,
    parameter [7:0] RED_GREEN_MARGIN = 8'd20,
    parameter [7:0] RED_BLUE_MARGIN = 8'd30,
    // Append new parameters to preserve legacy positional parameter overrides.
    parameter CARTOON_ENABLE=1,
    parameter CARTOON_COLOR_BITS=3,
    parameter CARTOON_OUTLINE_ENABLE=0,
    parameter [4:0] CARTOON_EDGE_THRESHOLD=5'd4,
    parameter SUPPORT_MOTION=1,
    parameter [2:0] DEFAULT_FLAGS=3'b111,
    parameter MOTION_SS_X=2,MOTION_SS_Y=4,MOTION_MIN_SAMPLES=4,
    parameter ROI_REFRESH_PERIOD=8,ROI_PAD=128,
    parameter ADAPTIVE_LIGHT=0, BBOX_MIN_BOX_AREA=512,
    parameter MOTION_FRAME_PERIOD=1, TARGETS=4, CB=(TARGETS<8?3:4), CARTOON_ROUND_ENABLE=2, DISPLAY_LIFT=0, MOTION_COLOR_DELTA=2, FIXED_TARGET_HUD=0, NOISE_REJECT=0,
    parameter TRAFFIC_ENABLE=0, SENSOR_DENOISE=0, UI_STYLE=0, FIELD_FIX=0, VISUAL_PROFILE=0,
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720,MOTION_PRE_AWB=DETECT_PRE_AWB,COORD_SCALE_SHIFT=0,EXTERNAL_TRACKER=0,EXTERNAL_MOTION=0,TIGHT_WEAK=0
)(
    output O_traffic_update,
    output [8:0] O_traffic_sample, // {mode_ok,good,occupied,count_a,count_b}
    input axi4s_video_aclk,I_rst_n,
    input I_motion_enable,I_target_enable,I_cartoon_enable,
    input awb_O_tvalid,awb_O_tuser,awb_O_tlast,
    input [95:0] awb_raw_data,awb_O_tdata,
    input [19:0] awb_gain_r,awb_gain_b,
    input [7:0] I_scene_luma,
    output [95:0] O_data,
    output O_valid,O_start,O_last,
    output O_motion_ready,O_motion_bbox_valid,
    output [15:0] O_motion_xmin,O_motion_xmax,O_motion_ymin,O_motion_ymax,O_motion_samples,
    output O_roi_full_scan,
    output [2:0] O_active_flags,
    input I_edge_enable,output O_edge_active,output [95:0] O_original_data,output [3:0] O_graphics_mask,output [3:0] O_edge_mask,
    output [CB-1:0] O_det_count,output O_det_valid,O_det_limit,O_det_role,O_det_text_ready,
    output [4:0] O_det_reason,output [TARGETS*16-1:0] O_det_xmin,O_det_xmax,O_det_ymin,O_det_ymax,
    output [TARGETS*2-1:0] O_det_shapes,output [TARGETS*8-1:0] O_det_ids,output [23:0] O_camera_vectors,output [3:0] O_analysis_seed,input [7:0] I_frame,output [7:0] O_det_frame,
    input I_ext_motion_ready,I_ext_motion_valid,input [15:0] I_ext_mx0,I_ext_mx1,I_ext_my0,I_ext_my1,I_ext_samples
);
    // Filter masks only; delay original pixels and markers by the same beat.
    wire [95:0] color_debug_data;
    wire [95:0] diagnostic_data;
    wire [95:0] filtered_data;
    wire [3:0] filtered_mask;
    wire filtered_valid,filtered_start,filtered_last;
    reg [7:0] ccl_frame_id;
    always @(posedge axi4s_video_aclk or negedge I_rst_n)if(!I_rst_n)ccl_frame_id<=0;else if(filtered_start)ccl_frame_id<=I_frame;
    assign O_det_frame=ccl_frame_id;
    wire [95:0] bbox_debug_data;
    wire [95:0] target_info_data;
    wire [95:0] probe_display_data;
    wire [3:0] red_mask,seed_mask,front_seed,filtered_seed;
    assign O_analysis_seed=seed_mask;
    reg [3:0] seed_delay1,seed_delay2;
    always @(posedge axi4s_video_aclk or negedge I_rst_n)
        if(!I_rst_n)begin seed_delay1<=0;seed_delay2<=0;end
        else begin seed_delay1<=seed_mask;seed_delay2<=seed_delay1;end
    assign front_seed=seed_delay2;
    wire [3:0] aligned_raw_mask;
    wire [95:0] detection_rgb=(DETECT_PRE_AWB!=0)?awb_raw_data:awb_O_tdata;
    wire [95:0] display_rgb=(DISPLAY_PRE_AWB!=0)?awb_raw_data:awb_O_tdata;
    color_segment_96 #(
        .TIGHT_WEAK(TIGHT_WEAK),.STRICT_RED_SEED(FIELD_FIX>=3),.WEAK_EXTENSION(VISUAL_PROFILE),.DISPLAY_MODE(0),.ADAPTIVE_LIGHT(ADAPTIVE_LIGHT),
        .RED_MIN(RED_MIN),
        .RED_GREEN_MARGIN(RED_GREEN_MARGIN),
        .RED_BLUE_MARGIN(RED_BLUE_MARGIN)
    ) u_color_segment_96 (
        .I_scene_luma(I_scene_luma),.I_tdata(detection_rgb),
        .I_tvalid(awb_O_tvalid),
        .O_tdata(),
        .O_seed_mask(seed_mask),.O_red_mask(red_mask)
    );
    wire [2:0] request_flags,front_flags,filtered_flags;
    frame_feature_flags #(.DEFAULT_FLAGS(DEFAULT_FLAGS)) u_feature_flags (
        .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),.I_start(awb_O_tuser),
        .I_requested({I_motion_enable,I_target_enable,I_cartoon_enable}),.O_flags(request_flags));
    wire [95:0] front_data;
    wire [3:0] front_red_mask;
    wire front_valid,front_start,front_last,front_motion;
    if(EXTERNAL_MOTION!=0)begin:g_external_motion
    // Keep the exact old two-register RGB/mask/flags/marker latency. Only the
    // motion analysis side branch changes; red segmentation/ROI stay aligned.
    reg v1,s1,l1,v2,s2,l2;reg [95:0] rgb1,rgb2;reg [3:0] red1,red2;reg [2:0] flags1,flags2;
    always @(posedge axi4s_video_aclk or negedge I_rst_n)
      if(!I_rst_n)begin v1<=0;s1<=0;l1<=0;v2<=0;s2<=0;l2<=0;rgb1<=0;rgb2<=0;red1<=0;red2<=0;flags1<=0;flags2<=0;end
      else begin v1<=awb_O_tvalid;s1<=awb_O_tuser;l1<=awb_O_tlast;rgb1<=display_rgb;red1<=red_mask;flags1<=request_flags;
        v2<=v1;s2<=s1;l2<=l1;rgb2<=rgb1;red2<=red1;flags2<=flags1;end
    assign front_valid=v2;assign front_start=s2;assign front_last=l2;assign front_data=rgb2;assign front_red_mask=red2;assign front_flags=flags2;assign front_motion=0;
    assign O_motion_ready=I_ext_motion_ready;assign O_motion_bbox_valid=I_ext_motion_valid;
    assign O_motion_xmin=I_ext_mx0;assign O_motion_xmax=I_ext_mx1;assign O_motion_ymin=I_ext_my0;assign O_motion_ymax=I_ext_my1;assign O_motion_samples=I_ext_samples;
    assign O_camera_vectors=0;
    end else if(NOISE_REJECT==0 || SUPPORT_MOTION==0)begin:g_original_motion
    assign O_camera_vectors=0;
    motion_rgb332_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.SUPPORT_MOTION(SUPPORT_MOTION),.DEFAULT_FLAGS(DEFAULT_FLAGS),
        .COLOR_DELTA(MOTION_COLOR_DELTA),.FRAME_PERIOD(MOTION_FRAME_PERIOD),.SS_X(MOTION_SS_X),.SS_Y(MOTION_SS_Y),.MIN_SAMPLES(MOTION_MIN_SAMPLES)) u_motion (
        .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
        .I_valid(awb_O_tvalid),.I_start(awb_O_tuser),.I_last(awb_O_tlast),
        .I_data(display_rgb),.I_detect_rgb(MOTION_PRE_AWB?awb_raw_data:detection_rgb),.I_red_mask(red_mask),.I_flags(request_flags),
        .O_data(front_data),.O_red_mask(front_red_mask),.O_flags(front_flags),
        .O_valid(front_valid),.O_start(front_start),.O_last(front_last),.O_motion_mask(front_motion),
        .O_ready(O_motion_ready),.O_bbox_valid(O_motion_bbox_valid),
        .O_xmin(O_motion_xmin),.O_xmax(O_motion_xmax),.O_ymin(O_motion_ymin),.O_ymax(O_motion_ymax),
        .O_sample_count(O_motion_samples)
    );
    end else begin:g_compensated_motion
    motion_compensated_96 #(.CAMERA_DETAIL(FIELD_FIX>=4),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.SAMPLE_FILTER(SENSOR_DENOISE),.SUPPORT_MOTION(SUPPORT_MOTION),.DEFAULT_FLAGS(DEFAULT_FLAGS),
        .COLOR_DELTA(1),.FRAME_PERIOD(1),.SS_X(1),.SS_Y(4),.MIN_SAMPLES(MOTION_MIN_SAMPLES)) u_motion (
        .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
        .I_valid(awb_O_tvalid),.I_start(awb_O_tuser),.I_last(awb_O_tlast),
        .I_data(display_rgb),.I_detect_rgb(MOTION_PRE_AWB?awb_raw_data:detection_rgb),.I_red_mask(red_mask),.I_flags(request_flags),
        .O_data(front_data),.O_red_mask(front_red_mask),.O_flags(front_flags),
        .O_valid(front_valid),.O_start(front_start),.O_last(front_last),.O_motion_mask(front_motion),
        .O_ready(O_motion_ready),.O_bbox_valid(O_motion_bbox_valid),
        .O_xmin(O_motion_xmin),.O_xmax(O_motion_xmax),.O_ymin(O_motion_ymin),.O_ymax(O_motion_ymax),
        .O_sample_count(O_motion_samples),.O_camera_vectors(O_camera_vectors));
    end
    wire [3:0] front_mask=front_flags[1]?front_red_mask:{4{front_motion}};
    reg edge_request,edge_pending,edge_display;
    wire edge_front=awb_O_tuser?I_edge_enable:edge_request;
    wire edge_commit=(MASK_FILTER_ENABLE!=0)?edge_pending:edge_front;
    always @(posedge axi4s_video_aclk or negedge I_rst_n)
        if(!I_rst_n)begin edge_request<=0;edge_pending<=0;edge_display<=0;end
        else begin
            if(awb_O_tuser)edge_request<=I_edge_enable;
            if(front_start)edge_pending<=edge_front;
            if(filtered_start)edge_display<=edge_commit;
        end
    
    assign O_edge_active=filtered_start?edge_commit:edge_display;
    assign O_original_data=filtered_data;
    reg [2:0] pending_flags;
    always @(posedge axi4s_video_aclk or negedge I_rst_n)
        if(!I_rst_n)pending_flags<=DEFAULT_FLAGS;
        else if(front_start)pending_flags<=front_flags;
    if (MASK_FILTER_ENABLE != 0) begin : g_mask_filter
        wire main_valid,main_start,main_last;wire [95:0] main_data;wire [3:0] main_mask,main_raw,main_seed;wire [31:0] bottom_gray;
        reg [2:0] cleanup_flags;
        always @(posedge axi4s_video_aclk or negedge I_rst_n)
            if(!I_rst_n)cleanup_flags<=DEFAULT_FLAGS;
            else if(main_start)cleanup_flags<=pending_flags;
        wire [2:0] cleanup_mode=main_start?pending_flags:cleanup_flags;
        mask_cleanup_switch_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.RGB_BLOCK_RAM(1),.SEED_ENABLE(VISUAL_PROFILE),.MOTION_MIN_NEIGHBORS(NOISE_REJECT!=0?3:1)) u_mask_cleanup_96 (
            .I_clk(axi4s_video_aclk), .I_rst_n(I_rst_n),
            .I_valid(front_valid), .I_frame_start(front_start), .I_last(front_last),
            .I_seed_mask(front_seed),.O_seed_mask(main_seed),.O_bottom_gray(bottom_gray),.I_data(front_data), .I_mask(front_mask),.I_majority(VISUAL_PROFILE!=0?cleanup_mode[1]:filtered_flags[1]),
            .O_data(main_data),
            .O_mask(main_mask), .O_valid(main_valid),.O_raw_mask(main_raw),
            .O_frame_start(main_start), .O_last(main_last)
        );
        if(VISUAL_PROFILE==2)begin:g_shared_spatial
            visual_neighborhood_96 u_neighborhood(
                .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),.I_valid(main_valid),.I_start(main_start),.I_last(main_last),
                .I_data(main_data),.I_bottom_gray(bottom_gray),.I_cartoon_enable(cleanup_mode[0]),
                .I_mask(main_mask),.I_raw_mask(main_raw),.I_seed_mask(main_seed),
                .O_valid(filtered_valid),.O_start(filtered_start),.O_last(filtered_last),.O_data(filtered_data),
                .O_mask(filtered_mask),.O_raw_mask(aligned_raw_mask),.O_seed_mask(filtered_seed),.O_edges(O_edge_mask));
        end else begin:g_direct
            if(VISUAL_PROFILE==3)begin:g_motion_edges
                wire [3:0] next_edges;
                reg v,s,l;reg [95:0] rgb;reg [3:0] mask,raw_mask,seed,edges;
                // Cut the scalar-gradient -> four-lane run collector -> FIFO
                // critical path. ALL parallel metadata shares this register.
                analysis_edges4 #(.IMG_WIDTH(IMG_WIDTH)) u_edges(.I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),.I_valid(main_valid),.I_start(main_start),.I_last(main_last),.I_data(main_data),.I_bottom_gray(bottom_gray),.O_edges(next_edges));
                always @(posedge axi4s_video_aclk or negedge I_rst_n)
                  if(!I_rst_n)begin v<=0;s<=0;l<=0;rgb<=0;mask<=0;raw_mask<=0;seed<=0;edges<=0;end
                  else begin v<=main_valid;s<=main_start;l<=main_last;rgb<=main_data;mask<=main_mask;raw_mask<=main_raw;seed<=main_seed;edges<=next_edges;end
                assign filtered_valid=v;assign filtered_start=s;assign filtered_last=l;
                assign filtered_data=rgb;assign filtered_mask=mask;assign aligned_raw_mask=raw_mask;assign filtered_seed=seed;assign O_edge_mask=edges;
            end else begin:g_no_edges
                assign filtered_valid=main_valid;assign filtered_start=main_start;assign filtered_last=main_last;
                assign filtered_data=main_data;assign filtered_mask=main_mask;assign aligned_raw_mask=main_raw;assign filtered_seed=main_seed;
                assign O_edge_mask=0;
            end
        end
    end else begin : g_no_mask_filter
        assign O_edge_mask=0;assign filtered_seed=front_seed;
        assign aligned_raw_mask=front_mask;
        assign filtered_data=front_data;
        assign filtered_mask=front_mask;
        assign filtered_valid=front_valid;
        assign filtered_start=front_start;
        assign filtered_last=front_last;
    end
    reg [2:0] display_flags;
    wire [2:0] commit_flags=(MASK_FILTER_ENABLE!=0)?pending_flags:front_flags;
    always @(posedge axi4s_video_aclk or negedge I_rst_n)
        if(!I_rst_n)display_flags<=DEFAULT_FLAGS;
        else if(filtered_start)display_flags<=commit_flags;
    assign filtered_flags=filtered_start?commit_flags:display_flags;
    assign O_active_flags=filtered_flags;
    wire target_active=filtered_flags[1];
    wire target_rst_n=I_rst_n && target_active;
    wire target_valid=filtered_valid && target_active;
    wire [95:0] styled_data;
    wire [95:0] quantized_data;
    genvar lift_channel;
    for(lift_channel=0;lift_channel<12;lift_channel=lift_channel+1)begin:g_display_lift
        wire [7:0] value=quantized_data[lift_channel*8+:8];
        // Modest gamma-like, piecewise lift. Monotone; 0/255 unchanged;
        // +25% below mid-gray, smoothly decreasing lift above it (max31).
        // Display only: raw segmentation and motion history never see this.
        wire [7:0] lift=(value[7]?~value:value)>>(FIELD_FIX!=0?1:2);
        // Quantize first: preserve the original +/-4 neutral-color guard.
        assign styled_data[lift_channel*8+:8]=VISUAL_PROFILE!=0?value:DISPLAY_LIFT!=0?value+lift:value;
    end
    if(CARTOON_ENABLE!=0 && VISUAL_PROFILE==0)begin:g_cartoon
        cartoon_96 #(.FIELD_FIX(FIELD_FIX),.COLOR_BITS(FIELD_FIX!=0?4:CARTOON_COLOR_BITS),.ROUND_ENABLE(CARTOON_ROUND_ENABLE),
            .OUTLINE_ENABLE(CARTOON_OUTLINE_ENABLE),
            .EDGE_THRESHOLD(CARTOON_EDGE_THRESHOLD)) u_cartoon (
            .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
            .I_valid(filtered_valid),.I_frame_start(filtered_start),.I_last(filtered_last),
            .I_enable(filtered_flags[0]),.I_data(filtered_data),.O_data(quantized_data),
            .O_valid(),.O_frame_start(),.O_last()
        );
    end else begin:g_no_cartoon
        assign quantized_data=filtered_data;
    end
    wire [95:0] motion_display_data;
    motion_display_96 u_motion_display(
        .I_valid(filtered_valid),.I_motion_enable(VISUAL_PROFILE!=0?1'b0:filtered_flags[2]),
        .I_target_enable(target_active),.I_mask(filtered_mask),.I_data(styled_data),
        .O_data(motion_display_data));
    genvar lane;
    for (lane=0;lane<4;lane=lane+1) begin : g_mask_display
        assign color_debug_data[lane*24 +:24]=
            (COLOR_DEBUG_MODE==2)?(filtered_mask[lane]?24'hffffff:24'h000000):
            (COLOR_DEBUG_MODE==1 && filtered_mask[lane])?24'h00ffff:
            motion_display_data[lane*24 +:24];
    end
    mask_diagnostic_96 #(.ENABLE(MASK_DIAGNOSTIC_ENABLE)) u_mask_diagnostic_96 (
        .I_valid(filtered_valid), .I_data(color_debug_data),
        .I_raw_mask(aligned_raw_mask), .I_clean_mask(filtered_mask),
        .O_data(diagnostic_data)
    );
    if (BBOX_ENABLE != 0) begin : g_bbox
        wire box_valid;
        wire [15:0] box_xmin,box_xmax,box_ymin,box_ymax,pixel_x,pixel_y;
        if (MULTI_TARGET_ENABLE != 0) begin : g_multi
            wire [CB-1:0] count,component_count;
            wire result_valid;
            wire component_active=target_active || (EXTERNAL_MOTION==0 && VISUAL_PROFILE!=0 && filtered_flags[2]);
            wire component_rst_n=I_rst_n && component_active;
            wire component_valid=filtered_valid && component_active;
            reg frame_role,result_role,result_ready;
            always @(posedge axi4s_video_aclk or negedge component_rst_n)
                if(!component_rst_n)begin frame_role<=0;result_role<=0;result_ready<=0;end
                else begin
                    if(filtered_start)frame_role<=target_active;
                    if(result_valid)begin result_role<=frame_role;result_ready<=1;end
                end
            // CCL results belong to the COMPLETED frame, not the current
            // switch state. Never draw motion results as target IDs or vice versa.
            wire same_role=VISUAL_PROFILE==0 || (result_ready && result_role==target_active);
            assign count=same_role?component_count:{CB{1'b0}};
            wire capacity_limit;
            wire [4:0] frame_reason;
            wire [TARGETS*16-1:0] mx0,mx1,my0,my1;
            wire [TARGETS*2-1:0] shapes;
            wire text_ready;
            wire track_ready;
            wire [TARGETS*8-1:0] track_ids;
            wire [TARGETS*8-1:0] internal_track_ids;wire internal_track_ready;
            assign track_ids=EXTERNAL_TRACKER!=0?0:internal_track_ids;
            assign track_ready=EXTERNAL_TRACKER!=0?1'b1:internal_track_ready;
            wire [TARGETS*16-1:0] bcd_x,bcd_y;
            wire [95:0] multi_box_data;
            wire [3:0] detection_mask=BBOX_USE_RAW_MASK ? aligned_raw_mask : filtered_mask;
            wire [3:0] roi_mask;
            motion_roi_gate_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.TARGETS(TARGETS),.REFRESH_PERIOD(ROI_REFRESH_PERIOD),.MOTION_PAD(ROI_PAD),.TRACK_PAD(ROI_PAD)) u_roi (
                .I_clk(axi4s_video_aclk),.I_rst_n(target_rst_n),.I_valid(target_valid),.I_start(filtered_start),
                .I_motion_enable(filtered_flags[2]),.I_motion_ready(O_motion_ready),
                .I_motion_box_valid(O_motion_bbox_valid),
                .I_motion_xmin(O_motion_xmin),.I_motion_xmax(O_motion_xmax),
                .I_motion_ymin(O_motion_ymin),.I_motion_ymax(O_motion_ymax),
                .I_count(count),.I_xmin(mx0),.I_xmax(mx1),.I_ymin(my0),.I_ymax(my1),
                .I_reason(frame_reason),.I_x(pixel_x),.I_y(pixel_y),.I_mask(detection_mask),
                .O_mask(roi_mask),.O_full_scan(O_roi_full_scan));
            red_components_96 #(.SHARE_LAST_ROW(EXTERNAL_MOTION),.CACHE_SEEDS(EXTERNAL_MOTION),.FAST_EVENT_COUNT(FIELD_FIX>=4?1:0),.FAST_EVENT_PAYLOAD(FIELD_FIX>=4?1:0),.FAST_QUEUE_LIMIT(FIELD_FIX>=4?1:0),.CACHE_GEOMETRY(FIELD_FIX>=4?1:0),.LABELS(FIELD_FIX>=4?24:32),.MOTION_MIN_AREA(FIELD_FIX>=3?24:0),.MOTION_MIN_BOX_AREA(FIELD_FIX>=3?32:0),.TARGET_MIN_SEEDS(FIELD_FIX>=3?16:0),.STRONG_START(VISUAL_PROFILE==3),.SEED_COUNTS(VISUAL_PROFILE==3),.MOTION_EDGE_GUARD(VISUAL_PROFILE==3),.EVENT_MIN_RUN(VISUAL_PROFILE==3?2:1),.SEEDED(VISUAL_PROFILE),.STATS_DRAM(1),.TARGETS(TARGETS),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.MIN_AREA(BBOX_MIN_AREA),.MIN_BOX_AREA(BBOX_MIN_BOX_AREA)) u_components (
                .I_clk(axi4s_video_aclk),.I_rst_n(component_rst_n),
                .I_valid(component_valid),.I_frame_start(filtered_start),
                // The external motion backend owns every non-red observation.
                // With component_active target-only, these are exact red-path
                // constants, allowing unused motion CCL arithmetic to trim.
                .I_motion_only(EXTERNAL_MOTION!=0?1'b0:!target_active),
                .I_seed_mask(EXTERNAL_MOTION!=0?filtered_seed:target_active?filtered_seed:(VISUAL_PROFILE==3?filtered_mask:4'hf)),
                .I_mask(EXTERNAL_MOTION!=0?roi_mask:target_active?roi_mask:(VISUAL_PROFILE==3&&FIELD_FIX<3?(filtered_mask|O_edge_mask):filtered_mask)),
                .O_pixel_x(pixel_x),.O_pixel_y(pixel_y),.O_count(component_count),.O_limit(capacity_limit),
                .O_limit_reason(),
                .O_row_peak(),.O_labels(),.O_objects(),
                .O_dropped(),.O_frame_reason(frame_reason),.O_result_valid(result_valid),
                .O_xmin(mx0),.O_xmax(mx1),.O_ymin(my0),.O_ymax(my1)
            );
            assign O_det_count=component_count;assign O_det_valid=result_valid;assign O_det_limit=capacity_limit;assign O_det_reason=frame_reason;assign O_det_role=EXTERNAL_MOTION!=0?1'b1:frame_role;
            assign O_det_xmin=mx0;assign O_det_xmax=mx1;assign O_det_ymin=my0;assign O_det_ymax=my1;
            assign O_det_shapes=shapes;assign O_det_ids=track_ids;assign O_det_text_ready=text_ready&&track_ready;
            // Statistics tap only: no video data or marker is delayed/changed.
            if(TRAFFIC_ENABLE!=0) begin:g_traffic
                wire mode_ok=target_active && !filtered_flags[2] && !O_edge_active;
                assign O_traffic_sample[8]=mode_ok;
                traffic_region_stats #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.TARGETS(TARGETS),
                    .A_X0(IMG_WIDTH/20),.A_X1(IMG_WIDTH*3/8-1),.C_X0(IMG_WIDTH*3/8),.C_X1(IMG_WIDTH*5/8-1),.B_X0(IMG_WIDTH*5/8),.B_X1(IMG_WIDTH*19/20-1),.ROI_Y0(IMG_HEIGHT*2/9),.ROI_Y1(IMG_HEIGHT*8/9-1)) u_stats(
                    .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
                    .I_mode_good(mode_ok),.I_valid(target_valid),.I_start(filtered_start),
                    .I_x(pixel_x),.I_y(pixel_y),.I_mask(detection_mask),
                    .I_result_valid(result_valid),.I_limit(capacity_limit),
                    .I_reason(frame_reason),.I_count(count),
                    .I_xmin(mx0),.I_xmax(mx1),.I_ymin(my0),.I_ymax(my1),
                    .O_update(O_traffic_update),.O_good(O_traffic_sample[7]),
                    .O_occupied(O_traffic_sample[6]),
                    .O_count_a(O_traffic_sample[5:3]),.O_count_b(O_traffic_sample[2:0]));
            end else begin:g_no_traffic
                assign O_traffic_update=1'b0;
                assign O_traffic_sample=9'b001000000;
            end
            assign box_valid=count!=0;
            assign box_xmin=mx0[15:0];assign box_xmax=mx1[15:0];
            assign box_ymin=my0[15:0];assign box_ymax=my1[15:0];
            multi_target_overlay_96 #(.MOTION_STYLE(VISUAL_PROFILE),.UI_STYLE(UI_STYLE),.TARGETS(TARGETS),.FIXED_HUD(FIXED_TARGET_HUD)) u_multi_overlay (
                .I_valid(component_valid),.I_data(diagnostic_data),
                .I_motion_only(!target_active),.I_pixel_x(pixel_x),.I_pixel_y(pixel_y),.I_count(count),.I_limit(capacity_limit),
                .I_reason(5'd0),
                .I_row_peak(8'd0),.I_labels(6'd0),.I_objects(6'd0),
                .I_dropped(16'd0),.I_frame_reason(5'd0),
                .I_xmin(mx0),.I_xmax(mx1),.I_ymin(my0),.I_ymax(my1),.O_data(multi_box_data),.I_split(O_edge_active)
            );
            shape_moments_96 #(.COORD_SCALE_SHIFT(COORD_SCALE_SHIFT),.TARGETS(TARGETS),.ACCUM_ERAM(TARGETS>4?1:0)) u_shapes (
                .I_clk(axi4s_video_aclk),.I_rst_n(target_rst_n),
                .I_valid(target_valid),.I_start(filtered_start),
                .I_mask(roi_mask),
                .I_x(pixel_x),.I_y(pixel_y),.I_count(count),
                .I_xmin(mx0),.I_xmax(mx1),.I_ymin(my0),.I_ymax(my1),
                .I_reason(frame_reason),.O_shape(shapes),.O_bcd_x(bcd_x),.O_bcd_y(bcd_y),.O_text_ready(text_ready)
            );
            shape_labels_96 #(.UI_STYLE(UI_STYLE),.TARGETS(TARGETS),.FIXED_HUD(FIXED_TARGET_HUD)) u_shape_labels (
                .I_valid(target_valid),.I_ready(text_ready && track_ready),.I_data(multi_box_data),.I_x(pixel_x),.I_y(pixel_y),
                .I_count(count),.I_xmin(mx0),.I_xmax(mx1),.I_ymin(my0),.I_ymax(my1),
                .I_shape(shapes),.I_bcd_x(bcd_x),.I_bcd_y(bcd_y),.I_ids(track_ids),.O_data(bbox_debug_data),.I_split(O_edge_active)
            );
            target_tracks_96 #(.TARGETS(TARGETS)) u_tracks (
                .I_clk(axi4s_video_aclk),.I_rst_n(target_rst_n),.I_start(VISUAL_PROFILE==3?result_valid:filtered_start),
                .I_count(VISUAL_PROFILE==3?component_count:count),.I_xmin(mx0),.I_xmax(mx1),.I_ymin(my0),.I_ymax(my1),
                .I_reason(frame_reason),.O_ids(internal_track_ids),.O_ready(internal_track_ready),
                .I_observed({TARGETS{1'b1}}),.I_preferred_tags({(TARGETS*7){1'b0}}),.I_forget({TARGETS{1'b0}}),.I_clear(1'b0),
                .O_track_active(),.O_track_tags(),.O_track_xmin(),.O_track_xmax(),.O_track_ymin(),.O_track_ymax(),.O_track_observed()
            );
        end else begin : g_single
        assign O_traffic_update=0;assign O_traffic_sample=9'b001000000;
        assign O_roi_full_scan=1'b1;
        red_bbox_96 #(
            .IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT), .MIN_AREA(BBOX_MIN_AREA),.MIN_BOX_AREA(BBOX_MIN_BOX_AREA),
            .SMOOTH_ENABLE(BBOX_SMOOTH_ENABLE)
        ) u_red_bbox_96 (
            .I_clk(axi4s_video_aclk), .I_rst_n(target_rst_n),
            .I_frame_start(filtered_start), .I_valid(target_valid),
            .I_data(diagnostic_data),
            .I_mask(BBOX_USE_RAW_MASK ? aligned_raw_mask : filtered_mask), .O_data(bbox_debug_data),
            .O_box_valid(box_valid), .O_xmin(box_xmin), .O_xmax(box_xmax),
            .O_ymin(box_ymin), .O_ymax(box_ymax), .O_area(),
            .O_pixel_x(pixel_x), .O_pixel_y(pixel_y)
        );
        end
        // Keep fixed NO TARGET, but do not cover per-object labels with the
        // previous single-target HUD or draw a second unguarded center cross.
        if(MULTI_TARGET_ENABLE != 0) begin:g_multi_status
            wire [95:0] no_target_data;
            target_info_96 #(.ADAPT_LAYOUT(FIXED_TARGET_HUD),.HUD_X(608),.HUD_Y(176)) u_no_target (
                .I_valid(target_valid),.I_data(bbox_debug_data),
                .I_pixel_x(pixel_x),.I_pixel_y(pixel_y),.I_box_valid(1'b0),
                .I_xmin(16'd0),.I_xmax(16'd0),.I_ymin(16'd0),.I_ymax(16'd0),.O_data(no_target_data),.I_split(O_edge_active)
            );
            assign target_info_data=VISUAL_PROFILE!=0 && !target_active?bbox_debug_data:box_valid?bbox_debug_data:no_target_data;
        end else begin:g_single_status
        target_info_96 u_target_info_96 (
            .I_valid(target_valid), .I_data(bbox_debug_data),
            .I_pixel_x(pixel_x), .I_pixel_y(pixel_y), .I_box_valid(box_valid),
            .I_xmin(box_xmin), .I_xmax(box_xmax), .I_ymin(box_ymin), .I_ymax(box_ymax),
            .O_data(target_info_data)
        );
        end
    end else begin : g_no_bbox
        assign O_traffic_update=0;assign O_traffic_sample=9'b001000000;
        assign O_roi_full_scan=1'b1;
        assign bbox_debug_data = diagnostic_data;
        assign target_info_data = diagnostic_data;
    end
    // Statistics use exactly matched before/after pixels, not guessed delays.
    if(RGB_PROBE_ENABLE!=0) begin:g_rgb_probe
    rgb_probe_diag_96 #(.RED_MIN(RED_MIN),.RG_MARGIN(RED_GREEN_MARGIN),.RB_MARGIN(RED_BLUE_MARGIN),
        .USE_PRE(DETECT_PRE_AWB)) u_rgb_probe (
        .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
        .I_valid(awb_O_tvalid),.I_start(awb_O_tuser),
        .I_pre(awb_raw_data),.I_post(awb_O_tdata),
        .I_gain_r(awb_gain_r),.I_gain_b(awb_gain_b),
        .I_draw_valid(filtered_valid),.I_draw_start(filtered_start),
        .I_data(target_info_data),.O_data(probe_display_data)
    );
    end else begin:g_no_rgb_probe
        assign probe_display_data=target_info_data;
    end
    assign O_last  = filtered_last;
    assign O_start  = filtered_start;
    assign O_data  = probe_display_data;
    genvar graphics_lane;
    for(graphics_lane=0;graphics_lane<4;graphics_lane=graphics_lane+1)begin:g_graphics
        // Shared downstream outline must never erase a green mask, box or text.
        assign O_graphics_mask[graphics_lane]=probe_display_data[graphics_lane*24+:24]!=styled_data[graphics_lane*24+:24];
    end
    assign O_valid = filtered_valid;
endmodule

// Explicit motion-only renderer. No storage and no extra pixel latency.
// Target+motion uses ROI assistance and leaves target colors unpainted.
module motion_display_96(input I_valid,I_motion_enable,I_target_enable,
 input [3:0] I_mask,input [95:0] I_data,output [95:0] O_data);
 genvar n;
 generate for(n=0;n<4;n=n+1)begin:g_lane
   assign O_data[n*24+:24]=I_valid&&I_motion_enable&&!I_target_enable&&I_mask[n]
       ?24'h00ff00:I_data[n*24+:24];
 end endgenerate
endmodule
