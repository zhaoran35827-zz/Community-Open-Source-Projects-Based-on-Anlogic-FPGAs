module isp_top #(
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720,RAW_STAGE=0,HD_HYBRID=0,
    // 0: display full-color AWB output; 1: display Sobel edges.
    parameter EDGE_DISPLAY = 0,
    // Applies only to the color branch: 0 original, 1 cyan overlay, 2 mask.
    // Step 8 defaults to original color; mode 2 remains the mask build option.
    parameter COLOR_DEBUG_MODE = 0,
    // Use the aligned, pre-AWB RGB for color display on this resource-limited
    // device. 0 keeps the previous dynamic-AWB color build option.
    parameter DISPLAY_PRE_AWB = 0,
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
    parameter SUPPORT_MOTION=1,parameter [2:0] DEFAULT_FLAGS=3'b111,
    parameter ADAPTIVE_LIGHT=1, BBOX_MIN_BOX_AREA=512,
    parameter EDGE_SPLIT_ENABLE=0,MOTION_FRAME_PERIOD=1,
    parameter ALGORITHM_OSD_ENABLE=0, TARGETS=4, CARTOON_ROUND_ENABLE=2, SPLIT_LABELS=0,
    parameter SHARED_COLOR_EDGES=0,DISPLAY_LIFT=0,MOTION_FINE=0,NOISE_REJECT=0,OSD_SAFE_MARGIN=0,
    parameter TRAFFIC_ENABLE=0, SENSOR_DENOISE=0, UI_STYLE=0, FIELD_FIX=0, VISUAL_PROFILE=0,
    parameter DISPLAY_WIDTH=IMG_WIDTH,DISPLAY_HEIGHT=IMG_HEIGHT,ANALYSIS_DIV=2
) (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready,
    input I_motion_enable,
    input I_target_enable,I_cartoon_enable,
    output O_motion_ready,O_motion_bbox_valid,
    output [15:0] O_motion_xmin,O_motion_xmax,O_motion_ymin,O_motion_ymax,O_motion_samples,
    input I_edge_enable,
    output O_roi_full_scan,
    output O_traffic_update,output [8:0] O_traffic_sample
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

    wire packed_valid,packed_start,packed_last;
    wire         edge_O_tlast;  //synthesis keep
    wire         edge_O_tuser;  //synthesis keep
    wire [ 95:0] edge_O_tdata;  //synthesis keep
    wire         edge_O_tvalid;  //synthesis keep
    wire         edge_O_tready;  //synthesis keep

    assign awb_O_tready = edge_O_tready;
    assign edge_O_tready = O_tready;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = (EDGE_SPLIT_ENABLE!=0) ? packed_last : ((EDGE_DISPLAY != 0) ? awb_O_tlast : edge_O_tlast);
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
demosaic #(
    .IMG_HEIGHT          (IMG_HEIGHT),  // 图像高度
    .IMG_WIDTH           (IMG_WIDTH), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

wire [7:0] scene_luma;
wire [95:0] awb_raw_data;
wire [19:0] awb_gain_r,awb_gain_b;
awb #(
    .IMG_HEIGHT(IMG_HEIGHT),
    .IMG_WIDTH (IMG_WIDTH),
    .FIELD_FIX(FIELD_FIX),.FIXED_GAIN(AWB_FIXED_GAIN)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready),
    .O_scene_luma(scene_luma),.O_raw_data(awb_raw_data),.O_gain_r(awb_gain_r),.O_gain_b(awb_gain_b)
);

wire [95:0] original_data;
wire [3:0] graphics_mask;
wire [3:0] shared_edge_mask;
wire edge_active;
wire [2:0] active_flags;
wire [1:0] active_detector_leader;
generate
if(RAW_STAGE!=0)begin:g_raw_stage
 assign active_detector_leader=0;
 assign O_traffic_update=0;assign O_traffic_sample=0;
 assign shared_edge_mask=0;assign graphics_mask=0;assign original_data=awb_O_tdata;assign edge_active=0;assign active_flags=0;
 assign O_motion_ready=0;assign O_motion_bbox_valid=0;assign O_motion_xmin=0;assign O_motion_xmax=0;assign O_motion_ymin=0;assign O_motion_ymax=0;assign O_motion_samples=0;assign O_roi_full_scan=1;
 assign edge_O_tdata=awb_O_tdata;assign edge_O_tvalid=awb_O_tvalid;assign edge_O_tuser=awb_O_tuser;assign edge_O_tlast=awb_O_tlast;
end else if(HD_HYBRID!=0)begin:g_hd_hybrid
 assign graphics_mask=0;
 fusion_hd_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.DISPLAY_WIDTH(DISPLAY_WIDTH),.DISPLAY_HEIGHT(DISPLAY_HEIGHT),.ANALYSIS_DIV(ANALYSIS_DIV),.TARGETS(TARGETS),.MOTION_TRACKING(FIELD_FIX>=2),.OBJECT_MOTION(FIELD_FIX>=2),.VISUAL_PIPELINE(FIELD_FIX>=2)) u_hd(
 .I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),.I_valid(awb_O_tvalid),.I_start(awb_O_tuser),.I_last(awb_O_tlast),
 .I_raw(awb_raw_data),.I_color(awb_O_tdata),.I_gain_r(awb_gain_r),.I_gain_b(awb_gain_b),.I_luma(scene_luma),
 .I_flags({I_edge_enable,I_motion_enable,I_target_enable,I_cartoon_enable}),
 .O_valid(edge_O_tvalid),.O_start(edge_O_tuser),.O_last(edge_O_tlast),.O_data(edge_O_tdata),.O_original(original_data),.O_edges(shared_edge_mask),
 .O_active_flags(active_flags),.O_edge_active(edge_active),.O_motion_ready(O_motion_ready),.O_motion_bbox_valid(O_motion_bbox_valid),
 .O_motion_xmin(O_motion_xmin),.O_motion_xmax(O_motion_xmax),.O_motion_ymin(O_motion_ymin),.O_motion_ymax(O_motion_ymax),.O_motion_samples(O_motion_samples),.O_roi_full_scan(O_roi_full_scan),
 .O_traffic_update(O_traffic_update),.O_traffic_sample(O_traffic_sample),.O_detector_leader(active_detector_leader));
end else if (EDGE_DISPLAY != 0) begin : g_edge_display
assign active_detector_leader=0;
assign O_traffic_update=0;assign O_traffic_sample=9'b001000000;
assign shared_edge_mask=0;assign graphics_mask=0;assign original_data=awb_O_tdata;assign edge_active=1'b0;assign active_flags=0;
assign O_motion_ready=0;assign O_motion_bbox_valid=0;
assign O_motion_xmin=0;assign O_motion_xmax=0;assign O_motion_ymin=0;assign O_motion_ymax=0;
assign O_motion_samples=0;assign O_roi_full_scan=1;
edge_detect_96 #(
    .IMG_WIDTH (IMG_WIDTH),
    .IMG_HEIGHT(IMG_HEIGHT),
    .EDGE_TH   (11'd80)
) u_edge_detect_96 (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (awb_O_tlast),
    .I_tuser (awb_O_tuser),
    .I_tdata (awb_O_tdata),
    .I_tvalid(awb_O_tvalid),
    .I_tready(awb_O_tready),
    .O_tlast (edge_O_tlast),
    .O_tuser (edge_O_tuser),
    .O_tdata (edge_O_tdata),
    .O_tvalid(edge_O_tvalid),
    .O_tready(edge_O_tready)
);
end else begin : g_color_display
    assign active_detector_leader=active_flags[1]?2'd1:active_flags[2]?2'd2:2'd0;
    fusion_vision_96 #(
        .VISUAL_PROFILE(VISUAL_PROFILE),.FIELD_FIX(FIELD_FIX),.TRAFFIC_ENABLE(TRAFFIC_ENABLE),.SENSOR_DENOISE(SENSOR_DENOISE),.UI_STYLE(UI_STYLE),
        .COLOR_DEBUG_MODE(COLOR_DEBUG_MODE),.DISPLAY_PRE_AWB(DISPLAY_PRE_AWB),
        .DETECT_PRE_AWB(DETECT_PRE_AWB),.RGB_PROBE_ENABLE(RGB_PROBE_ENABLE),
        .BBOX_ENABLE(BBOX_ENABLE),.MULTI_TARGET_ENABLE(MULTI_TARGET_ENABLE),
        .MASK_FILTER_ENABLE(MASK_FILTER_ENABLE),.MASK_DIAGNOSTIC_ENABLE(MASK_DIAGNOSTIC_ENABLE),
        .BBOX_USE_RAW_MASK(BBOX_USE_RAW_MASK),.BBOX_SMOOTH_ENABLE(BBOX_SMOOTH_ENABLE),.BBOX_MIN_AREA(BBOX_MIN_AREA),
        .RED_MIN(RED_MIN),.RED_GREEN_MARGIN(RED_GREEN_MARGIN),.RED_BLUE_MARGIN(RED_BLUE_MARGIN),
        .TARGETS(TARGETS),.CARTOON_ROUND_ENABLE(CARTOON_ROUND_ENABLE),.CARTOON_ENABLE(CARTOON_ENABLE),.CARTOON_COLOR_BITS(CARTOON_COLOR_BITS),
        .CARTOON_OUTLINE_ENABLE(SHARED_COLOR_EDGES!=0?0:CARTOON_OUTLINE_ENABLE),
        .DISPLAY_LIFT(DISPLAY_LIFT),.FIXED_TARGET_HUD(SHARED_COLOR_EDGES),.MOTION_SS_X(MOTION_FINE!=0?1:2),.MOTION_SS_Y(MOTION_FINE!=0?2:4),
        .MOTION_COLOR_DELTA(MOTION_FINE!=0?1:2),.CARTOON_EDGE_THRESHOLD(CARTOON_EDGE_THRESHOLD),
        .SUPPORT_MOTION(SUPPORT_MOTION),.DEFAULT_FLAGS(DEFAULT_FLAGS),
        .ADAPTIVE_LIGHT(ADAPTIVE_LIGHT),.BBOX_MIN_BOX_AREA(BBOX_MIN_BOX_AREA),
        .MOTION_FRAME_PERIOD(MOTION_FRAME_PERIOD),.NOISE_REJECT(NOISE_REJECT)
    ) u_fusion (
        .O_traffic_update(O_traffic_update),.O_traffic_sample(O_traffic_sample),
        .axi4s_video_aclk(axi4s_video_aclk),.I_rst_n(I_rst_n),
        .I_motion_enable(I_motion_enable),.I_target_enable(I_target_enable),.I_cartoon_enable(I_cartoon_enable),
        .I_edge_enable(EDGE_SPLIT_ENABLE!=0 ? I_edge_enable : 1'b0),
        .O_edge_active(edge_active),.O_original_data(original_data),.O_edge_mask(shared_edge_mask),.O_graphics_mask(graphics_mask),
        .awb_O_tvalid(awb_O_tvalid),.awb_O_tuser(awb_O_tuser),.awb_O_tlast(awb_O_tlast),
        .I_scene_luma(scene_luma),.awb_raw_data(awb_raw_data),.awb_O_tdata(awb_O_tdata),.awb_gain_r(awb_gain_r),.awb_gain_b(awb_gain_b),
        .O_data(edge_O_tdata),.O_valid(edge_O_tvalid),.O_start(edge_O_tuser),.O_last(edge_O_tlast),
        .O_motion_ready(O_motion_ready),.O_motion_bbox_valid(O_motion_bbox_valid),
        .O_motion_xmin(O_motion_xmin),.O_motion_xmax(O_motion_xmax),.O_motion_ymin(O_motion_ymin),.O_motion_ymax(O_motion_ymax),
        .O_motion_samples(O_motion_samples),.O_roi_full_scan(O_roi_full_scan),.O_active_flags(active_flags)
    );

end
endgenerate

wire [95:0] packed_rgb;

generate if(EDGE_SPLIT_ENABLE!=0 && RAW_STAGE==0)begin:g_split
    edge_split_96 #(.IMG_WIDTH(DISPLAY_WIDTH),.IMG_HEIGHT(DISPLAY_HEIGHT),.VISUAL_PROFILE(VISUAL_PROFILE),.FIELD_FIX(FIELD_FIX),.REFINED_EDGES(SENSOR_DENOISE),.COLOR_EDGES(SHARED_COLOR_EDGES),.NOISE_REJECT(NOISE_REJECT),
        .SHARED_OUTLINE(SHARED_COLOR_EDGES!=0?CARTOON_OUTLINE_ENABLE:0)) u_split(.I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
        .I_valid(edge_O_tvalid),.I_start(edge_O_tuser),.I_last(edge_O_tlast),
        .I_precomputed_edges(shared_edge_mask),.I_enable(edge_active),.I_data(edge_O_tdata),.I_original(original_data),
        .I_cartoon_enable(active_flags[0]),.I_graphics_mask(graphics_mask),
        .O_valid(packed_valid),.O_start(packed_start),.O_last(packed_last),.O_data(packed_rgb));
end else begin:g_no_split
    assign packed_rgb=edge_O_tdata;assign packed_valid=edge_O_tvalid;
    assign packed_start=edge_O_tuser;assign packed_last=edge_O_tlast;
end endgenerate
// Capture mode at the SAME processed SOF as the split line cache input.
// The pending tag may advance during the preceding last-row flush; the OSD
// snapshots it only at its own output SOF, so the previous frame stays coherent.
wire [95:0] osd_rgb;
generate if(ALGORITHM_OSD_ENABLE!=0 && RAW_STAGE==0)begin:g_algorithm_osd
    reg [3:0] pending_mode;
    reg [1:0] pending_leader;
    wire [3:0] input_mode={edge_active,active_flags};
    always @(posedge axi4s_video_aclk or negedge I_rst_n)
        if(!I_rst_n)begin pending_mode<=0;pending_leader<=0;end
        else if(edge_O_tuser)begin pending_mode<=input_mode;pending_leader<=active_detector_leader;end
    algorithm_osd_96 #(.ROLE_AWARE(HD_HYBRID!=0&&FIELD_FIX>=2),.IMG_WIDTH(DISPLAY_WIDTH),.IMG_HEIGHT(DISPLAY_HEIGHT),.HEADER_Y(VISUAL_PROFILE!=0?64:16),.UI_STYLE(UI_STYLE),.SPLIT_LABELS(SPLIT_LABELS),.PANEL_X(OSD_SAFE_MARGIN!=0?64:16),.HEADER_MARGIN(OSD_SAFE_MARGIN!=0?64:16)) u_osd(.I_clk(axi4s_video_aclk),.I_rst_n(I_rst_n),
        .I_valid(packed_valid),.I_start(packed_start),.I_data(packed_rgb),
        .I_mode(edge_O_tuser?input_mode:pending_mode),.I_leader(edge_O_tuser?active_detector_leader:pending_leader),.O_data(osd_rgb));
end else begin:g_no_algorithm_osd
    assign osd_rgb=packed_rgb;
end endgenerate
    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),
	
        .I_96b_frame_start  ( packed_start         ),
        .I_96b_valid        ( packed_valid         ),
        .I_96b_data         ( osd_rgb              ),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule

// Fixed 16x16 ROI: x=624..639, y=352..367. Sample the earliest pixel of
// each four-pixel beat: 64 matched samples/frame. No mask gating or tracking.
module rgb_probe_diag_96 #(
    parameter RED_MIN=80,RG_MARGIN=30,RB_MARGIN=30,USE_PRE=0,
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720
) (
    input I_clk,I_rst_n,I_valid,I_start,
    input [95:0] I_pre,I_post,
    input [19:0] I_gain_r,I_gain_b,
    input I_draw_valid,I_draw_start,input [95:0] I_data,
    output [95:0] O_data
);
    reg [10:0] x,draw_x;reg [9:0] y,draw_y;reg start_d,draw_start_d;
    wire sof=I_start && !start_d;
    wire dsof=I_draw_start && !draw_start_d;
    wire [10:0] px=sof?11'd0:x,dx=dsof?11'd0:draw_x;
    wire [9:0] py=sof?10'd0:y,dy=dsof?10'd0:draw_y;
    wire sample=I_valid && px>=624 && px<640 && py>=352 && py<368;
    wire first_sample=px==624 && py==352;
    wire last_sample=px==636 && py==367;
    wire [7:0] pr=I_pre[95:88],pg=I_pre[87:80],pb=I_pre[79:72];
    wire [7:0] ar=I_post[95:88],ag=I_post[87:80],ab=I_post[79:72];
    // Nine-bit comparisons exactly match color_segment_96; no overflow.
    wire [7:0] dr=(USE_PRE!=0)?pr:ar,dg=(USE_PRE!=0)?pg:ag,db=(USE_PRE!=0)?pb:ab;
    wire bad_r=dr<RED_MIN;
    wire bad_g={1'b0,dr}<({1'b0,dg}+RG_MARGIN);
    wire bad_b={1'b0,dr}<({1'b0,db}+RB_MARGIN);
    reg [13:0] sr,sg,sb,tr,tg,tb;
    reg [6:0] fr,fg,fb,passes,n;
    wire [13:0] nsr=(first_sample?14'd0:sr)+pr;
    wire [13:0] nsg=(first_sample?14'd0:sg)+pg;
    wire [13:0] nsb=(first_sample?14'd0:sb)+pb;
    wire [13:0] ntr=(first_sample?14'd0:tr)+ar;
    wire [13:0] ntg=(first_sample?14'd0:tg)+ag;
    wire [13:0] ntb=(first_sample?14'd0:tb)+ab;
    wire [6:0] nfr=(first_sample?7'd0:fr)+bad_r;
    wire [6:0] nfg=(first_sample?7'd0:fg)+bad_g;
    wire [6:0] nfb=(first_sample?7'd0:fb)+bad_b;
    wire [6:0] np=(first_sample?7'd0:passes)+!(bad_r||bad_g||bad_b);
    wire [6:0] nn=(first_sample?7'd0:n)+7'd1;
    reg [23:0] pre_avg,post_avg;reg [6:0] fail_r,fail_g,fail_b,pass_count;
    reg [19:0] gain_r,gain_b;reg fresh;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x<=0;y<=0;draw_x<=0;draw_y<=0;start_d<=0;draw_start_d<=0;
            sr<=0;sg<=0;sb<=0;tr<=0;tg<=0;tb<=0;fr<=0;fg<=0;fb<=0;passes<=0;n<=0;
            pre_avg<=0;post_avg<=0;fail_r<=0;fail_g<=0;fail_b<=0;pass_count<=0;
            gain_r<=0;gain_b<=0;fresh<=0;
        end else begin
            start_d<=I_start;draw_start_d<=I_draw_start;
            if(sof) begin x<=0;y<=0;end
            if(I_valid) begin
                if(px==IMG_WIDTH-4)begin x<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;end
                else begin x<=px+4;y<=py;end
            end
            if(dsof)begin draw_x<=0;draw_y<=0;end
            if(I_draw_valid)begin
                if(dx==IMG_WIDTH-4)begin draw_x<=0;draw_y<=dy==IMG_HEIGHT-1?0:dy+1'b1;end
                else begin draw_x<=dx+4;draw_y<=dy;end
            end
            if(sof)begin n<=0;fresh<=0;end
            if(sample) begin
                sr<=nsr;sg<=nsg;sb<=nsb;tr<=ntr;tg<=ntg;tb<=ntb;
                fr<=nfr;fg<=nfg;fb<=nfb;passes<=np;n<=nn;
                if(last_sample && nn==64) begin
                    pre_avg<={nsr[13:6],nsg[13:6],nsb[13:6]};
                    post_avg<={ntr[13:6],ntg[13:6],ntb[13:6]};
                    fail_r<=nfr;fail_g<=nfg;fail_b<=nfb;pass_count<=np;
                    gain_r<=I_gain_r;gain_b<=I_gain_b;fresh<=1;
                end
            end
        end
    end
    // Decimal RGB/counts; gain is Q16 hexadecimal (10000 = 1.0).
    function [11:0] bcd8;
        input [7:0] value;integer i,j;reg [11:0] b;
        begin b=0;for(i=7;i>=0;i=i-1)begin
            for(j=0;j<3;j=j+1)if(b[j*4+:4]>=5)b[j*4+:4]=b[j*4+:4]+3;
            b={b[10:0],value[i]};end bcd8=b;end
    endfunction
    function [7:0] hexchar;input [3:0] v;
        begin hexchar=v<10?"0"+v:"A"+v-10;end
    endfunction
    wire [35:0] pre_bcd={bcd8(pre_avg[23:16]),bcd8(pre_avg[15:8]),bcd8(pre_avg[7:0])};
    wire [35:0] post_bcd={bcd8(post_avg[23:16]),bcd8(post_avg[15:8]),bcd8(post_avg[7:0])};
    wire [35:0] fail_bcd={bcd8({1'b0,fail_r}),bcd8({1'b0,fail_g}),bcd8({1'b0,fail_b})};
    wire [11:0] pass_bcd=bcd8({1'b0,pass_count});
    function [7:0] character;
        input [2:0] line;input [4:0] col;reg [35:0] numbers;
        begin
            character=" ";numbers=0;
            if(line==0)begin
                case(col)
                0:character="R";1:character="G";2:character="B";
                4:character="P";5:character="R";6:character="O";7:character="B";8:character="E";
                10:character=USE_PRE?"P":"D";11:character=USE_PRE?"R":"E";12:character=USE_PRE?"E":"C";
                14:character=fresh?"O":"W";15:character=fresh?"K":"T";endcase
            end else if(line<=3)begin
                numbers=line==1?pre_bcd:line==2?post_bcd:fail_bcd;
                case(col)
                0:character=line==3?"F":"P";
                1:character=line==1?"R":line==2?"O":"A";
                2:character=line==1?"E":line==2?"S":"I";
                3:character=line==2?"T":line==3?"L":" ";
                5:character="R";6:character=":";
                7:character="0"+numbers[35:32];8:character="0"+numbers[31:28];9:character="0"+numbers[27:24];
                11:character="G";12:character=":";
                13:character="0"+numbers[23:20];14:character="0"+numbers[19:16];15:character="0"+numbers[15:12];
                17:character="B";18:character=":";
                19:character="0"+numbers[11:8];20:character="0"+numbers[7:4];21:character="0"+numbers[3:0];endcase
            end else if(line==4)begin
                case(col)
                0:character="P";1:character="A";2:character="S";3:character="S";4:character=":";
                5:character="0"+pass_bcd[11:8];6:character="0"+pass_bcd[7:4];7:character="0"+pass_bcd[3:0];
                9:character="O";10:character="F";12:character="0";13:character="6";14:character="4";endcase
            end else if(line==5)begin
                case(col)
                0:character="G";1:character="A";2:character="I";3:character="N";
                5:character="R";6:character=":";
                7:character=hexchar(gain_r[19:16]);8:character=hexchar(gain_r[15:12]);
                9:character=hexchar(gain_r[11:8]);10:character=hexchar(gain_r[7:4]);11:character=hexchar(gain_r[3:0]);
                13:character="B";14:character=":";
                15:character=hexchar(gain_b[19:16]);16:character=hexchar(gain_b[15:12]);
                17:character=hexchar(gain_b[11:8]);18:character=hexchar(gain_b[7:4]);19:character=hexchar(gain_b[3:0]);endcase
            end
        end
    endfunction
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1)begin:g_render
        wire [10:0] rx=dx+lane,lx=rx-64;
        wire [9:0] ly=dy-480;
        wire panel=rx>=64 && rx<416 && dy>=480 && dy<576;
        wire roi=rx>=624 && rx<640 && dy>=352 && dy<368;
        wire border=roi && (rx==624 || rx==639 || dy==352 || dy==367);
        wire [15:0] bits;
        osd_char_lib font(.I_char(character(ly[6:4],lx[8:4])),.I_row(ly[3:0]),.O_row_bits(bits));
        assign O_data[(3-lane)*24+:24]=!I_draw_valid?I_data[(3-lane)*24+:24]:
            panel?(bits[15-lx[3:0]]?24'hffff00:24'h202020):border?24'h0080ff:I_data[(3-lane)*24+:24];
    end endgenerate
endmodule
