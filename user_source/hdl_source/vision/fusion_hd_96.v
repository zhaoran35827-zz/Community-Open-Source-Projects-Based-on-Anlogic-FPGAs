`timescale 1ns/1ps
// Local image operations stay at full resolution. Only analysis is decimated.
// The two streams never share a valid counter or stall the capture transport.
module fusion_hd_96 #(parameter IMG_WIDTH=1920,IMG_HEIGHT=1080,TARGETS=6,CB=3,MOTION_TOP_PAD=8,DISPLAY_WIDTH=IMG_WIDTH,DISPLAY_HEIGHT=IMG_HEIGHT,ANALYSIS_DIV=2,MOTION_TRACKING=0,OBJECT_MOTION=0,VISUAL_PIPELINE=0)(
 input I_clk,I_rst_n,I_valid,I_start,I_last,input [95:0] I_raw,I_color,
 input [19:0] I_gain_r,I_gain_b,input [7:0] I_luma,input [3:0] I_flags,
 output O_valid,O_start,O_last,output [95:0] O_data,O_original,
 output [3:0] O_edges,output [2:0] O_active_flags,output O_edge_active,
 output O_motion_ready,O_motion_bbox_valid,O_roi_full_scan,
 output [15:0] O_motion_xmin,O_motion_xmax,O_motion_ymin,O_motion_ymax,O_motion_samples,
 output O_traffic_update,output [8:0] O_traffic_sample,output [1:0] O_detector_leader);
 localparam MOTION_TARGETS=TARGETS<3?TARGETS:3;
 localparam AW=IMG_WIDTH/ANALYSIS_DIV,AH=IMG_HEIGHT/ANALYSIS_DIV;
 localparam NEW_MOTION=MOTION_TRACKING!=0&&OBJECT_MOTION!=0,MPAD=NEW_MOTION?0:MOTION_TOP_PAD;
 localparam DIRECT_MAP=ANALYSIS_DIV==2&&DISPLAY_WIDTH==IMG_WIDTH;
 localparam AXB=$clog2(AW),AYB=$clog2(AH);

 reg [3:0] frame_flags;
 wire [1:0] detector_leader;wire motion_primary,priority_epoch;
 detector_priority priority_control(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(I_start),.I_flags(I_flags),.O_leader(detector_leader),.O_motion_primary(motion_primary),.O_epoch_changed(priority_epoch));
 wire [23:0] camera_vectors;wire [3:0] analysis_seeds;
 wire [3:0] mode=I_start?I_flags:frame_flags;
 wire primary_target=MOTION_TRACKING!=0?(detector_leader==1):mode[1];
 wire motion_assist=MOTION_TRACKING!=0&&detector_leader==2&&mode[1];
 assign O_detector_leader=MOTION_TRACKING!=0?detector_leader:(mode[1]?2'd1:mode[2]?2'd2:2'd0);
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)frame_flags<=0;else if(I_start)frame_flags<=I_flags;
 wire dv,ds,dl;wire [95:0] drgb;
 wire av,asof,al;wire [95:0] corrected,araw;
 reg [7:0] analysis_frame_counter;
 wire [7:0] current_frame=asof?analysis_frame_counter+1'b1:analysis_frame_counter;
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)analysis_frame_counter<=0;else if(MOTION_TRACKING!=0&&asof)analysis_frame_counter<=analysis_frame_counter+1'b1;
 wire [7:0] result_frame;reg [7:0] motion_frame;reg [4:0] motion_reason;

 generate if(ANALYSIS_DIV==1)begin:g_full_analysis
  assign dv=0;assign ds=0;assign dl=0;assign drgb=0;
  assign av=I_valid;assign asof=I_start;assign al=I_last;assign corrected=I_color;assign araw=I_raw;
 end else begin:g_half_analysis
 hd_downsample2_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) down(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(I_valid),.I_start(I_start),.I_last(I_last),.I_data(I_raw),.O_valid(dv),.O_start(ds),.O_last(dl),.O_data(drgb));
 hd_analysis_color_96 correction(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(dv),.I_start(ds),.I_last(dl),.I_data(drgb),
 .I_gain_r(I_gain_r[17:10]),.I_gain_b(I_gain_b[17:10]),.O_valid(av),.O_start(asof),.O_last(al),.O_data(corrected),.O_raw(araw));
 end endgenerate
 wire object_valid,object_ready,object_limit,object_box,object_assist;wire [7:0] object_frame;
 wire [CB-1:0] object_count,object_moving_count;wire [TARGETS-1:0] object_observed;
 wire [TARGETS*16-1:0] ox0,ox1,oy0,oy1,omx0,omx1,omy0,omy1;
 wire [15:0] obx0,obx1,oby0,oby1;
 generate if(NEW_MOTION)begin:g_object_motion
  wire [MOTION_TARGETS*16-1:0] bx0,bx1,by0,by1;
  wire [MOTION_TARGETS-1:0] observed;
  assign ox0={{(TARGETS-MOTION_TARGETS)*16{1'b0}},bx0};
  assign ox1={{(TARGETS-MOTION_TARGETS)*16{1'b0}},bx1};
  assign oy0={{(TARGETS-MOTION_TARGETS)*16{1'b0}},by0};
  assign oy1={{(TARGETS-MOTION_TARGETS)*16{1'b0}},by1};
  assign omx0=ox0;assign omx1=ox1;assign omy0=oy0;assign omy1=oy1;
  assign object_observed={{(TARGETS-MOTION_TARGETS){1'b0}},observed};
  motion_objects_96 #(.WIDTH(AW),.HEIGHT(AH),.TARGETS(MOTION_TARGETS),.CB(CB),.SINGLE_OUTPUT(1),.RECORDS(12),.CAMERA_SKIP_FLAT(1),.CONTOUR_GUARD(1)) motion(
   .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(I_start&&(priority_epoch||!I_flags[2])),.I_enable(mode[2]),.I_assist(motion_assist),
   .I_valid(av),.I_start(asof),.I_rgb(araw),.I_frame(current_frame),.O_valid(object_valid),.O_ready(object_ready),.O_limited(object_limit),.O_frame(object_frame),
   .O_assist(object_assist),.O_count(object_count),.O_moving_count(object_moving_count),.O_x0(bx0),.O_x1(bx1),.O_y0(by0),.O_y1(by1),.O_mx0(),.O_mx1(),.O_my0(),.O_my1(),.O_observed(observed),
   .O_bbox_valid(object_box),.O_bx0(obx0),.O_bx1(obx1),.O_by0(oby0),.O_by1(oby1),.O_cycles(),.O_busy());
 end else begin:g_legacy_object_outputs
  assign object_assist=0;assign object_valid=0;assign object_ready=0;assign object_limit=0;assign object_box=0;assign object_frame=0;assign object_count=0;assign object_moving_count=0;assign object_observed=0;
  assign ox0=0;assign ox1=0;assign oy0=0;assign oy1=0;assign omx0=0;assign omx1=0;assign omy0=0;assign omy1=0;assign obx0=0;assign obx1=0;assign oby0=0;assign oby1=0;
 end endgenerate
 wire [CB-1:0] count;wire result_valid,limited,role,text_ready;
 wire [4:0] reason;wire [TARGETS*16-1:0] ax0,ax1,ay0,ay1;
 wire [TARGETS*2-1:0] shapes;wire [TARGETS*8-1:0] ids,analysis_ids;
 wire [15:0] mx0,mx1,my0,my1;
 fusion_vision_96 #(.IMG_WIDTH(AW),.IMG_HEIGHT(AH),.VISUAL_PROFILE(3),.FIELD_FIX(MOTION_TRACKING!=0?4:3),.NOISE_REJECT(1),.SENSOR_DENOISE(1),
 .TIGHT_WEAK(1),.EXTERNAL_MOTION(NEW_MOTION),.EXTERNAL_TRACKER(MOTION_TRACKING),.TARGETS(TARGETS),.TRAFFIC_ENABLE(1),.DEFAULT_FLAGS(0),.CARTOON_ENABLE(0),.CARTOON_OUTLINE_ENABLE(0),
 .ADAPTIVE_LIGHT(1),.DETECT_PRE_AWB(0),.MOTION_PRE_AWB(1),.DISPLAY_PRE_AWB(1),.BBOX_MIN_AREA(ANALYSIS_DIV==1?100:48),.BBOX_MIN_BOX_AREA(ANALYSIS_DIV==1?512:128),.ROI_PAD(ANALYSIS_DIV==1?96:64)) analysis(
 .I_ext_motion_ready(object_ready),.I_ext_motion_valid(object_box),.I_ext_mx0(obx0),.I_ext_mx1(obx1),.I_ext_my0(oby0),.I_ext_my1(oby1),.I_ext_samples({{(14-CB){1'b0}},object_moving_count,2'b00}),
 .axi4s_video_aclk(I_clk),.I_rst_n(I_rst_n),.awb_O_tvalid(av),.awb_O_tuser(asof),.awb_O_tlast(al),
 .awb_raw_data(araw),.awb_O_tdata(corrected),.awb_gain_r(I_gain_r),.awb_gain_b(I_gain_b),.I_scene_luma(I_luma),
 .I_target_enable(primary_target),.I_motion_enable(mode[2]),.I_cartoon_enable(1'b0),.I_edge_enable(mode[3]),
 .O_motion_ready(O_motion_ready),.O_motion_bbox_valid(O_motion_bbox_valid),.O_motion_xmin(mx0),.O_motion_xmax(mx1),.O_motion_ymin(my0),.O_motion_ymax(my1),.O_motion_samples(O_motion_samples),.O_roi_full_scan(O_roi_full_scan),
 .O_traffic_update(O_traffic_update),.O_traffic_sample(O_traffic_sample),
 .O_data(),.O_valid(),.O_start(),.O_last(),.O_active_flags(),.O_edge_active(),.O_original_data(),.O_graphics_mask(),.O_edge_mask(),
 .O_det_count(count),.O_det_valid(result_valid),.O_det_limit(limited),.O_det_role(role),.O_det_reason(reason),.O_det_text_ready(text_ready),
 .O_det_xmin(ax0),.O_det_xmax(ax1),.O_det_ymin(ay0),.O_det_ymax(ay1),.O_det_shapes(shapes),.O_det_ids(analysis_ids),.O_camera_vectors(camera_vectors),.O_analysis_seed(analysis_seeds),.I_frame(current_frame),.O_det_frame(result_frame));
 assign O_motion_xmin=map_low(mx0);assign O_motion_xmax=map_high(mx1);
 assign O_motion_ymin=map_low(my0);assign O_motion_ymax=map_high(my1);
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)begin motion_frame<=0;motion_reason<=0;end else if(result_valid&&!role)begin motion_frame<=result_frame;motion_reason<=reason;end
 wire [TARGETS-1:0] db_active,db_observed;wire [TARGETS*7-1:0] db_tags;
 wire [TARGETS*16-1:0] dbx0,dbx1,dby0,dby1;
 wire motion_done;wire [CB-1:0] motion_count;wire [TARGETS*16-1:0] cmx0,cmx1,cmy0,cmy1;
 generate if(!NEW_MOTION)begin:g_legacy_confirmation
 motion_box_confirm #(.FAST_RAW_GEOMETRY(MOTION_TRACKING),.ERAM_CLUSTER(MOTION_TRACKING),.TARGETS(TARGETS),.CB(CB),.IMG_WIDTH(AW),.IMG_HEIGHT(AH),.MERGE_GAP(MOTION_TRACKING!=0?32:12),.DIAGONAL_MERGE(MOTION_TRACKING),.COHERENT_MERGE(MOTION_TRACKING),.CAMERA_RELATIVE(MOTION_TRACKING),.TEMPORAL_VOTES(MOTION_TRACKING!=0?2:0),.ADAPTIVE_GAP(MOTION_TRACKING),.OWNER_GUARD(MOTION_TRACKING),.MATCH_DISTANCE(64),.MOVE_PIXELS(3),.MIN_CLUSTER_SPAN(8),.COORD_BITS($clog2(AW)>$clog2(AH)?$clog2(AW):$clog2(AH))) confirm_motion(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(I_start&&(!I_flags[2]||primary_target||(MOTION_TRACKING!=0&&priority_epoch))),
 .I_valid(result_valid&&!role),.I_count(reason[4:1]==0?count:{CB{1'b0}}),.I_xmin(ax0),.I_xmax(ax1),.I_ymin(ay0),.I_ymax(ay1),
 .I_camera_vectors(camera_vectors),.I_owner_active(db_active),.I_owner_x0(dbx0),.I_owner_x1(dbx1),.I_owner_y0(dby0),.I_owner_y1(dby1),.O_valid(motion_done),.O_count(motion_count),.O_xmin(cmx0),.O_xmax(cmx1),.O_ymin(cmy0),.O_ymax(cmy1));
 end else begin:g_reconstructed_confirmation
  assign motion_done=0;assign motion_count=0;assign cmx0=0;assign cmx1=0;assign cmy0=0;assign cmy1=0;
 end endgenerate
 wire joined_valid,joined_limited,shared_track_ready;
 wire [CB-1:0] joined_count;wire [TARGETS-1:0] joined_observed,joined_expired;wire [TARGETS*7-1:0] joined_hints;
 wire [TARGETS*16-1:0] jx0,jx1,jy0,jy1;

 wire motion_clear=detector_leader!=2 || !mode[1] || (I_start&&priority_epoch);
 generate if(NEW_MOTION)begin:g_reconstructed_motion
  assign joined_valid=object_valid&&object_assist==motion_assist;assign joined_count=motion_assist?object_count:object_moving_count;
  assign joined_observed=motion_assist?object_observed:{TARGETS{1'b1}};
  assign joined_expired=0;assign joined_hints=0;assign joined_limited=object_limit;
  assign jx0=ox0;assign jx1=ox1;assign jy0=oy0;assign jy1=oy1;
 end else if(MOTION_TRACKING!=0)begin:g_persistent_motion
  wire persist_valid,persist_limited;wire [CB-1:0] persist_count;
  wire [TARGETS-1:0] persist_observed,persist_expired;wire [TARGETS*7-1:0] persist_hints;
  wire [TARGETS*16-1:0] persist_x0,persist_x1,persist_y0,persist_y1;
  wire appearance_valid;wire [7:0] appearance_frame;wire [TARGETS-1:0] visible;
  wire [TARGETS*7-1:0] visible_tags;wire [TARGETS*16-1:0] vx0,vx1,vy0,vy1;
  track_appearance_96 #(.IMG_WIDTH(AW),.IMG_HEIGHT(AH),.TARGETS(TARGETS)) appearance(
   .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(motion_clear),.I_valid(av),.I_start(asof),.I_data(araw),.I_red_seed(analysis_seeds),.I_red_assist(mode[1]),.I_camera_vectors(camera_vectors),
   .I_active(db_active),.I_observed(db_observed),.I_tags(db_tags),.I_xmin(dbx0),.I_xmax(dbx1),.I_ymin(dby0),.I_ymax(dby1),
   .O_valid(appearance_valid),.O_visible(visible),.O_tags(visible_tags),.O_xmin(vx0),.O_xmax(vx1),.O_ymin(vy0),.O_ymax(vy1),.I_db_ready(shared_track_ready),.I_frame(current_frame),.O_frame(appearance_frame));
  motion_observation_join #(.TARGETS(TARGETS),.CB(CB),.IMG_WIDTH(AW),.IMG_HEIGHT(AH)) observations(
   .I_tracker_ready(shared_track_ready),.I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(motion_clear),.I_motion_valid(motion_done),.I_motion_frame(motion_frame),.I_motion_count(motion_count),.I_motion_reason(motion_reason),
   .I_mx0(cmx0),.I_mx1(cmx1),.I_my0(cmy0),.I_my1(cmy1),.I_appearance_valid(appearance_valid),.I_appearance_frame(appearance_frame),.I_visible(visible),.I_appearance_tags(visible_tags),
   .I_ax0(vx0),.I_ax1(vx1),.I_ay0(vy0),.I_ay1(vy1),.I_track_active(db_active),.I_track_tags(db_tags),.I_track_observed(db_observed),.I_dbx0(dbx0),.I_dbx1(dbx1),.I_dby0(dby0),.I_dby1(dby1),.O_hints(persist_hints),.O_expired(persist_expired),
   .O_valid(persist_valid),.O_count(persist_count),.O_observed(persist_observed),.O_limited(persist_limited),.O_frame(),.O_x0(persist_x0),.O_x1(persist_x1),.O_y0(persist_y0),.O_y1(persist_y1));
  // Motion alone reports current detections. Only an explicitly enabled red
  // assistant permits appearance-maintained tracks after the object stops.
  assign joined_valid=motion_assist?persist_valid:motion_done;
  assign joined_count=motion_assist?persist_count:motion_count;
  assign joined_observed=motion_assist?persist_observed:0;
  assign joined_expired=motion_assist?persist_expired:0;
  assign joined_hints=motion_assist?persist_hints:0;
  assign joined_limited=motion_assist?persist_limited:motion_reason!=0;
  assign jx0=motion_assist?persist_x0:cmx0;assign jx1=motion_assist?persist_x1:cmx1;
  assign jy0=motion_assist?persist_y0:cmy0;assign jy1=motion_assist?persist_y1:cmy1;
 end else begin:g_nonpersistent_motion
  assign joined_valid=motion_done;assign joined_count=motion_count;assign joined_observed=0;assign joined_hints=0;assign joined_expired=0;assign joined_limited=limited;
  assign jx0=cmx0;assign jx1=cmx1;assign jy0=cmy0;assign jy1=cmy1;
 end endgenerate
 // A detector may finish its old frame after the primary changes at SOF.
 // Never let that late opposite-role result populate the new track database.
 wire red_result=result_valid&&role&&(MOTION_TRACKING==0||primary_target);
 wire accepted_motion=joined_valid&&(MOTION_TRACKING==0||detector_leader==2);
 wire result_event=red_result||accepted_motion;
 wire event_role=red_result;
 wire [CB-1:0] event_count=event_role?count:joined_count;
 wire event_limit=event_role?limited:joined_limited;
 wire [TARGETS*16-1:0] ex0=(MOTION_TRACKING!=0?primary_target:event_role)?ax0:jx0,ex1=(MOTION_TRACKING!=0?primary_target:event_role)?ax1:jx1,ey0=(MOTION_TRACKING!=0?primary_target:event_role)?ay0:jy0,ey1=(MOTION_TRACKING!=0?primary_target:event_role)?ay1:jy1;

 wire [TARGETS*16-1:0] direct_y0;
 genvar map_slot;
 generate for(map_slot=0;map_slot<TARGETS;map_slot=map_slot+1)begin:g_direct_y
  wire [AYB-1:0] original_y=ey0[map_slot*16+:AYB];
  wire [AYB-1:0] padded_y=original_y>=MPAD?original_y-MPAD:0;
  assign direct_y0[map_slot*16+:16]={{(15-AYB){1'b0}},(event_role?original_y:padded_y),1'b0};
 end endgenerate
 generate if(NEW_MOTION)begin:g_red_id_ram
  // Motion coordinates/appearance already have their own ERAM records. No
  // motion consumer reads this red-ID database; shape labels are red-only.
  red_ids_eram #(.TARGETS(TARGETS),.CB(CB)) tracks(
   .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(!primary_target||(I_start&&priority_epoch)),.I_start(red_result),
   .I_count(count),.I_x0(ax0),.I_x1(ax1),.I_y0(ay0),.I_y1(ay1),.I_reason(reason),.O_ids(ids),.O_ready(shared_track_ready),.O_active(),.O_tags());
  assign db_active=0;assign db_observed=0;assign db_tags=0;assign dbx0=0;assign dbx1=0;assign dby0=0;assign dby1=0;
 end else if(MOTION_TRACKING!=0)begin:g_shared_tracker
  wire track_clear=detector_leader==0||(!primary_target&&!motion_assist)||(I_start&&priority_epoch);
  target_tracks_96 #(.TARGETS(TARGETS),.DB_META(1),.LIVE_BOX_INPUTS(1),.BOX_DB(1)) tracks(
   .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(track_clear),.I_start(result_event),.I_count(event_count),.I_xmin(ex0),.I_xmax(ex1),.I_ymin(ey0),.I_ymax(ey1),.I_reason(event_role?reason:5'd0),
   .O_ids(ids),.O_ready(shared_track_ready),.I_observed(event_role?{TARGETS{1'b1}}:joined_observed),.I_preferred_tags(event_role?{(TARGETS*7){1'b0}}:joined_hints),.I_forget(event_role?{TARGETS{1'b0}}:joined_expired),
   .O_track_active(db_active),.O_track_tags(db_tags),.O_track_xmin(dbx0),.O_track_xmax(dbx1),.O_track_ymin(dby0),.O_track_ymax(dby1),.O_track_observed(db_observed));
 end else begin:g_internal_tracker_outputs
  assign ids=analysis_ids;assign shared_track_ready=1;assign db_active=0;assign db_observed=0;assign db_tags=0;assign dbx0=0;assign dbx1=0;assign dby0=0;assign dby1=0;
 end endgenerate
 // Analysis retains the1920x1080 source, independently of display reduction.
 // This branch reduces only corrected RGB for720P presentation.
 wire display_v,display_s,display_l;wire [95:0] display_rgb;
 generate if(DISPLAY_WIDTH==IMG_WIDTH)begin:g_same_display
  assign display_v=I_valid;assign display_s=I_start;assign display_l=I_last;assign display_rgb=I_color;
 end else begin:g_720_display
  display_scale_2of3_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_scale(
   .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(I_valid),.I_start(I_start),.I_last(I_last),.I_data(I_color),
   .O_valid(display_v),.O_start(display_s),.O_last(display_l),.O_data(display_rgb));
 end endgenerate
 reg [3:0] display_flags;reg display_target_primary;
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)display_target_primary<=0;else if(display_s)display_target_primary<=primary_target;
 wire [3:0] visual_flags=display_s?mode:display_flags;
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)display_flags<=0;else if(display_s)display_flags<=mode;
 // Bounded exact division for960x540 analysis coordinates. No DSP or divider IP.
 function [9:0] third;input [9:0] v;integer i;reg [1:0] rem;reg [2:0] next;
 begin third=0;rem=0;for(i=9;i>=0;i=i-1)begin next={rem,v[i]};
  if(next>=3)begin third[i]=1;rem=next-3;end else rem=next[1:0];end end endfunction
 function [15:0] map_low;input [15:0] v;
 begin map_low=ANALYSIS_DIV==1?v:DISPLAY_WIDTH==IMG_WIDTH?(v<<1):(v+third(v[9:0]));end endfunction
 function [15:0] map_high;input [15:0] v;
 begin map_high=ANALYSIS_DIV==1?v:DISPLAY_WIDTH==IMG_WIDTH?((v<<1)|16'd1):(v+third(v[9:0])+16'd1);end endfunction
 // Display center RGB row is shared by mask cleanup and spatial kernel.
 wire cv,cs,cl;wire [95:0] crgb;wire [31:0] bottom;
 mask_cleanup_switch_96 #(.IMG_WIDTH(DISPLAY_WIDTH),.IMG_HEIGHT(DISPLAY_HEIGHT),.RGB_BLOCK_RAM(1)) center(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(display_v),.I_frame_start(display_s),.I_last(display_l),.I_data(display_rgb),.I_mask(4'd0),.I_majority(1'b0),.I_seed_mask(4'd0),
 .O_mask(),.O_raw_mask(),.O_seed_mask(),.O_valid(cv),.O_frame_start(cs),.O_last(cl),.O_data(crgb),.O_bottom_gray(bottom));
 wire sv,ss,sl;wire [95:0] spatial;wire [3:0] edge_bits;
 visual_neighborhood_96 #(.IMG_WIDTH(DISPLAY_WIDTH),.IMG_HEIGHT(DISPLAY_HEIGHT)) spatial_kernel(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(cv),.I_start(cs),.I_last(cl),.I_data(crgb),.I_bottom_gray(bottom),.I_cartoon_enable(visual_flags[0]),
 .I_mask(4'd0),.I_raw_mask(4'd0),.I_seed_mask(4'd0),.O_mask(),.O_raw_mask(),.O_seed_mask(),.O_valid(sv),.O_start(ss),.O_last(sl),.O_data(spatial),.O_edges(edge_bits));
 // Break two font/overlay muxes in series. RGB, binary edges and all
 // video markers receive exactly the same one-clock stage in native mode.
 wire label_valid,label_sof,label_target;wire [15:0] label_x,label_y;wire [95:0] label_rgb;
 wire [95:0] boxes;

 assign O_active_flags=display_flags[2:0];assign O_edge_active=display_flags[3];
 // Results are published only at the FULL display SOF. A half-resolution
 // result strobe never becomes the full image SOF or a pixel-count reset.
 reg pending_valid,pending_role,pending_limit,pending_assist;reg [CB-1:0] pending_count;
 reg [TARGETS*16-1:0] px0,px1,py0,py1;
 reg [CB-1:0] show_count;reg show_limit,show_role,show_text;
 reg [TARGETS*16-1:0] sx0,sx1,sy0,sy1,bcdx,bcdy;
 reg [TARGETS*2-1:0] show_shapes;reg [TARGETS*8-1:0] show_ids;
 reg bcd_busy,bcd_ready;reg [3:0] bcd_step;reg [TARGETS*16-1:0] work_bcdx,work_bcdy;
 reg mapping_busy,mapping_good;reg [5:0] mapping_step;
 wire [2:0] mapping_slot=mapping_step[4:2];
 reg [15:0] mapping_input;
 always @* case(mapping_step[1:0])
  0:mapping_input=px0[mapping_slot*16+:16];1:mapping_input=px1[mapping_slot*16+:16];
  2:mapping_input=py0[mapping_slot*16+:16];default:mapping_input=py1[mapping_slot*16+:16];endcase
 wire [15:0] mapped=mapping_step[0]?map_high(mapping_input):map_low(mapping_input);
 wire [2:0] bcd_slot=bcd_step[3:1];
 wire [16:0] bcd_sum=bcd_step[0]?({1'b0,py0[bcd_slot*16+:16]}+{1'b0,py1[bcd_slot*16+:16]}):({1'b0,px0[bcd_slot*16+:16]}+{1'b0,px1[bcd_slot*16+:16]});
 function [15:0] to_bcd;input [11:0] value;integer n,k;reg [15:0] q;
 begin q=0;for(n=11;n>=0;n=n-1)begin for(k=0;k<4;k=k+1)if(q[k*4+:4]>=5)q[k*4+:4]=q[k*4+:4]+3;q={q[14:0],value[n]};end to_bcd=q;end endfunction
 wire [15:0] bcd_value=to_bcd(bcd_sum[12:1]);
 // One coordinate converter and one font serve both detector roles. Bind
 // assistance to the completed result; switches cannot relabel an old frame.
 wire pending_text_ready=bcd_ready && (pending_role?shared_track_ready:(pending_assist&&motion_assist));
 wire [TARGETS*8-1:0] pending_ids=pending_role?ids:{TARGETS{8'h80}};
 reg [10:0] x,y;wire [15:0] vx=ss?16'd0:{5'd0,x},vy=ss?16'd0:{5'd0,y};
 generate if(VISUAL_PIPELINE!=0)begin:g_visual_stage
  reg v,s,l;reg [95:0] rgb,raw;reg [3:0] edges;reg [15:0] x,y;
  always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)begin v<=0;s<=0;l<=0;rgb<=0;raw<=0;edges<=0;x<=0;y<=0;end
  else begin v<=sv;s<=ss;l<=sl;rgb<=boxes;raw<=spatial;edges<=edge_bits;x<=vx;y<=vy;end
  assign label_valid=v;assign label_sof=0;assign label_target=show_role;assign label_x=x;assign label_y=y;assign label_rgb=rgb;
  assign O_valid=v;assign O_start=s;assign O_last=l;assign O_original=raw;assign O_edges=edges;
 end else begin:g_direct_visual
  assign label_valid=sv;assign label_sof=ss;assign label_target=display_target_primary;assign label_x=vx;assign label_y=vy;assign label_rgb=boxes;
  assign O_valid=sv;assign O_start=ss;assign O_last=sl;assign O_original=spatial;assign O_edges=edge_bits;
 end endgenerate
 integer j;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin pending_valid<=0;pending_role<=0;pending_limit<=0;pending_assist<=0;pending_count<=0;px0<=0;px1<=0;py0<=0;py1<=0;
 show_count<=0;show_limit<=0;show_role<=0;show_text<=0;sx0<=0;sx1<=0;sy0<=0;sy1<=0;show_shapes<=0;show_ids<=0;bcdx<=0;bcdy<=0;x<=0;y<=0;bcd_busy<=0;bcd_ready<=0;bcd_step<=0;work_bcdx<=0;work_bcdy<=0;mapping_busy<=0;mapping_good<=0;mapping_step<=0;end
 else begin
  if(I_start && !I_flags[1]&&!I_flags[2])pending_valid<=0;
  if(result_valid&&reason[4:1]!=0&&(MOTION_TRACKING==0||role))begin pending_valid<=0;pending_limit<=1;end
  if(result_event)begin
   pending_valid<=(ANALYSIS_DIV==1||DIRECT_MAP)&&(!event_role||reason[4:1]==0);pending_role<=event_role;pending_count<=event_count;pending_limit<=event_limit;pending_assist<=!event_role&&motion_assist;
   mapping_busy<=ANALYSIS_DIV!=1&&!DIRECT_MAP;mapping_step<=0;mapping_good<=!event_role||reason[4:1]==0;
   bcd_busy<=(ANALYSIS_DIV==1||DIRECT_MAP)&&(event_role||motion_assist);bcd_ready<=0;bcd_step<=0;work_bcdx<=0;work_bcdy<=0;
   for(j=0;j<TARGETS;j=j+1)begin px0[j*16+:16]<=DIRECT_MAP?{{(15-AXB){1'b0}},ex0[j*16+:AXB],1'b0}:ex0[j*16+:16];px1[j*16+:16]<=DIRECT_MAP?{{(15-AXB){1'b0}},ex1[j*16+:AXB],1'b1}:ex1[j*16+:16];
    // Motion samples are four analysis rows apart; causal neighbour support
    // can discard the first sampled row. Restore the bounded leading edge
    // AFTER confirmation, so this does not invent motion or move its centroid
    // inside the temporal confirmation engine. Target boxes stay exact.
    py0[j*16+:16]<=DIRECT_MAP?direct_y0[j*16+:16]:event_role ? ey0[j*16+:16] :
       (ey0[j*16+:16]>=MPAD ? ey0[j*16+:16]-MPAD : 16'd0);
    py1[j*16+:16]<=DIRECT_MAP?{{(15-AYB){1'b0}},ey1[j*16+:AYB],1'b1}:ey1[j*16+:16];end
  end else if(mapping_busy)begin
   case(mapping_step[1:0])
    0:px0[mapping_slot*16+:16]<=mapped;1:px1[mapping_slot*16+:16]<=mapped;
    2:py0[mapping_slot*16+:16]<=mapped;3:py1[mapping_slot*16+:16]<=mapped;
   endcase
   if(mapping_step==TARGETS*4-1)begin mapping_busy<=0;pending_valid<=mapping_good;bcd_busy<=pending_role||pending_assist;end
   else mapping_step<=mapping_step+1'b1;
  end else if(bcd_busy)begin
   if(bcd_step[0])work_bcdy[bcd_slot*16+:16]<=bcd_value;else work_bcdx[bcd_slot*16+:16]<=bcd_value;
   if(bcd_step==TARGETS*2-1)begin bcd_busy<=0;bcd_ready<=1;end else bcd_step<=bcd_step+1'b1;
  end
  if(ss)begin
   show_count<=pending_valid && pending_role==display_target_primary && (display_flags[1]||display_flags[2])?pending_count:0;
   show_limit<=pending_limit && (display_flags[1]||display_flags[2]);show_role<=display_target_primary;show_text<=pending_text_ready&&pending_role==display_target_primary&&(display_flags[1]||display_flags[2]);
   sx0<=px0;sx1<=px1;sy0<=py0;sy1<=py1;show_shapes<=pending_role&&text_ready?shapes:0;show_ids<=pending_ids;bcdx<=work_bcdx;bcdy<=work_bcdy;
  end
  if(ss)begin x<=0;y<=0;end
  if(sv)begin if(vx==DISPLAY_WIDTH-4)begin x<=0;y<=vy==DISPLAY_HEIGHT-1?0:vy+1'b1;end else begin x<=vx+4;y<=vy;end end
 end
 // SOF pixel uses the to-be-latched bundle, avoiding one-beat old boxes.
 wire [CB-1:0] draw_count=ss?(pending_valid&&pending_role==display_target_primary&&(display_flags[1]||display_flags[2])?pending_count:0):show_count;
 
 target_overlay_compact #(.VALIDATED_INPUTS(MOTION_TRACKING),.TARGETS(TARGETS),.FIXED_HUD(1),.UI_STYLE(1),.MOTION_STYLE(1),.SHOW_MOTION_HUD(0),.IMG_WIDTH(DISPLAY_WIDTH),.IMG_HEIGHT(DISPLAY_HEIGHT)) renderer(
 .I_valid(sv&&(display_flags[1]||display_flags[2])),.I_data(spatial),.I_x(vx),.I_y(vy),.I_count(draw_count),.I_limit(ss?pending_limit:show_limit),.I_motion_only(!display_target_primary),.I_split(display_flags[3]),
 .I_xmin(ss?px0:sx0),.I_xmax(ss?px1:sx1),.I_ymin(ss?py0:sy0),.I_ymax(ss?py1:sy1),.O_data(boxes));
 shape_labels_96 #(.TARGETS(TARGETS),.FIXED_HUD(1),.UI_STYLE(1),.TRACK_TAGS(1)) labels(
 .I_valid(label_valid),.I_ready(label_sof?(pending_text_ready&&pending_role==display_target_primary&&(display_flags[1]||display_flags[2])):show_text),.I_data(label_rgb),.I_x(label_x),.I_y(label_y),.I_count(label_sof?draw_count:show_count),
 .I_xmin(label_sof?px0:sx0),.I_xmax(label_sof?px1:sx1),.I_ymin(label_sof?py0:sy0),.I_ymax(label_sof?py1:sy1),
 .I_shape(label_sof?(pending_role&&text_ready?shapes:{(TARGETS*2){1'b0}}):show_shapes),.I_bcd_x(label_sof?work_bcdx:bcdx),.I_bcd_y(label_sof?work_bcdy:bcdy),.I_ids(label_sof?pending_ids:show_ids),.I_split(display_flags[3]),.O_data(O_data));
endmodule

// Two-sided scalar contrast from the already existing center/bottom rows.
// Weak contours may connect a moving object's separate difference stripes;
// they cannot qualify a CCL result without accumulated true motion seeds.
module analysis_edges4 #(parameter IMG_WIDTH=960,THRESHOLD=14)(
 input I_clk,I_rst_n,I_valid,I_start,I_last,input [95:0] I_data,input [31:0] I_bottom_gray,output [3:0] O_edges);
 reg [8:0] x;reg [7:0] left_y;wire first=I_start||x==0;
 wire [31:0] gray;
 genvar n;
 generate for(n=0;n<4;n=n+1)begin:g_y
  wire [23:0] p=I_data[(3-n)*24+:24];wire [9:0] sum={2'd0,p[23:16]}+{1'd0,p[15:8],1'b0}+{2'd0,p[7:0]}+10'd2;
  assign gray[(3-n)*8+:8]=sum[9:2];
  wire [7:0] a;
  if(n==0)begin:g_first assign a=first?sum[9:2]:left_y;end
  else begin:g_previous assign a=gray[(4-n)*8+:8];end
  wire [7:0] b=I_bottom_gray[(3-n)*8+:8];
  wire [7:0] dx=a>=sum[9:2]?a-sum[9:2]:sum[9:2]-a;
  wire [7:0] dy=b>=sum[9:2]?b-sum[9:2]:sum[9:2]-b;
  assign O_edges[3-n]=I_valid&&(dx>=THRESHOLD||dy>=THRESHOLD);
 end endgenerate
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)begin x<=0;left_y<=0;end
 else begin if(I_start)x<=0;if(I_valid)begin x<=I_last?0:(I_start?1:x+1'b1);left_y<=gray[7:0];end end
endmodule
