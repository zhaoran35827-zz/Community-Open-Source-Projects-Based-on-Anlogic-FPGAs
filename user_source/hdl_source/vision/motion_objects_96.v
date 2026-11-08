`timescale 1ns/1ps
// Completed-frame side branch. It never supplies or gates main video markers.
// All work must finish before map reuse; deadline/epoch cancels fail closed.
module motion_objects_96 #(parameter WIDTH=640,HEIGHT=360,TARGETS=6,CB=3,
 COLS=WIDTH/4,ROWS=HEIGHT/4,AB=$clog2(COLS*ROWS),FRAME_BUDGET=WIDTH*HEIGHT*4,SINGLE_OUTPUT=0,RECORDS=16,CAMERA_SKIP_FLAT=0,CONTOUR_GUARD=0)(
 input I_clk,I_rst_n,I_clear,I_enable,I_assist,I_valid,I_start,
 input [95:0] I_rgb,input [7:0] I_frame,
 output reg O_valid,O_ready,O_limited,output O_assist,output reg [7:0] O_frame,
 output [CB-1:0] O_count,O_moving_count,
 output [TARGETS*16-1:0] O_x0,O_x1,O_y0,O_y1,O_mx0,O_mx1,O_my0,O_my1,
 output [TARGETS-1:0] O_observed,
 output reg O_bbox_valid,output reg [15:0] O_bx0,O_bx1,O_by0,O_by1,
 output reg [31:0] O_cycles,output O_busy);
 localparam IDLE=0,RESET_WAIT=1,CAMERA_START=2,CAMERA_WAIT=3,REGION_START=4,WORK_WAIT=5,UNION=6,PUBLISH=7;
 reg [2:0] state;reg [4:0] reset_wait;
 reg frame_enable,frame_assist,work_assist,work_bank;reg [7:0] work_frame;
 wire complete,complete_bank;wire [7:0] complete_frame;
 wire overrun=complete&&state!=IDLE;
 wire deadline=state!=IDLE&&state!=RESET_WAIT&&O_cycles>=FRAME_BUDGET;
 wire abort_work=I_clear||overrun||deadline;
 wire wm_enable,wm_write,wm_ready;wire [AB-1:0] wm_address;wire [17:0] wm_data,wm_q;
 motion_grid_pair_96 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) maps(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(I_clear),.I_valid(I_valid),.I_start(I_start),.I_rgb(I_rgb),.I_frame(I_frame),
  .O_complete(complete),.O_bank(complete_bank),.O_frame(complete_frame),
  .W_bank(work_bank),.W_enable(wm_enable),.W_write(wm_write),.W_address(wm_address),.W_data(wm_data),.W_q(wm_q),.W_write_ready(wm_ready));
 reg camera_begin,region_begin,tracks_begin;
 wire camera_busy,camera_done,camera_warm;
 wire cp_begin,cp_learn;wire [9:0] cp_template;wire signed [11:0] cp_x,cp_y;
 wire [47:0] camera_x,camera_y;wire [3:0] camera_supported;
 wire patch_done,patch_quality;wire signed [7:0] patch_dx,patch_dy;wire [15:0] patch_score,patch_zero;
 motion_camera_consensus #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SKIP_FLAT(CAMERA_SKIP_FLAT)) camera(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_begin(camera_begin),.I_abort(abort_work),.I_warm(camera_warm),
  .O_busy(camera_busy),.O_done(camera_done),.O_warm(camera_warm),.P_begin(cp_begin),.P_learn(cp_learn),.P_template(cp_template),.P_x(cp_x),.P_y(cp_y),
  .P_done(patch_done),.P_quality(patch_quality),.P_dx(patch_dx),.P_dy(patch_dy),.P_score(patch_score),.P_zero(patch_zero),.O_x(camera_x),.O_y(camera_y),.O_supported(camera_supported));
 wire region_busy,region_done,region_overflow,region_valid,region_ready,rm_enable,rm_write;
 wire [AB-1:0] rm_address;wire [17:0] rm_data;
 wire [$clog2(COLS):0] region_x0,region_x1;wire [$clog2(ROWS):0] region_y0,region_y1;
 wire [14:0] region_area;
 motion_region_worker #(.COLS(COLS),.ROWS(ROWS)) regions(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_begin(region_begin),.I_abort(abort_work),.O_busy(region_busy),.O_done(region_done),.O_overflow(region_overflow),
  .M_enable(rm_enable),.M_write(rm_write),.M_address(rm_address),.M_data(rm_data),.M_q(wm_q),.I_write_ready(wm_ready),
  .O_proposal(region_valid),.I_proposal_ready(region_ready),.O_x0(region_x0),.O_x1(region_x1),.O_y0(region_y0),.O_y1(region_y1),.O_area(region_area),.O_colour());
 wire tracks_busy,tracks_done,tracks_limited;wire [7:0] tracks_frame;
 wire [CB-1:0] count,moving_count;wire [15:0] track_bx0,track_bx1,track_by0,track_by1;
 assign O_assist=work_assist;
 wire tp_begin,tp_learn;wire [9:0] tp_template;wire signed [11:0] tx0,tx1,ty0,ty1;wire signed [7:0] tcx,tcy,ttx,tty;
 wire [7:0] gx0=region_x0,gx1=region_x1,gy0=region_y0,gy1=region_y1;
 motion_object_tracks #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.TARGETS(TARGETS),.CB(CB),.SINGLE_OUTPUT(SINGLE_OUTPUT),.RECORDS(RECORDS),.ROBUST_MATCH(CONTOUR_GUARD)) objects(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_begin(tracks_begin),.I_abort(abort_work),.I_assist(work_assist),.I_frame(work_frame),.I_camera_x(camera_x),.I_camera_y(camera_y),
  .G_valid(region_valid),.G_done(region_done),.G_overflow(region_overflow),.G_ready(region_ready),.G_x0(gx0),.G_x1(gx1),.G_y0(gy0),.G_y1(gy1),.G_area(region_area),
  .O_bx0(track_bx0),.O_bx1(track_bx1),.O_by0(track_by0),.O_by1(track_by1),.O_busy(tracks_busy),.O_done(tracks_done),.O_limited(tracks_limited),.O_frame(tracks_frame),.O_count(count),.O_moving_count(moving_count),
  .O_x0(O_x0),.O_x1(O_x1),.O_y0(O_y0),.O_y1(O_y1),.O_mx0(O_mx0),.O_mx1(O_mx1),.O_my0(O_my0),.O_my1(O_my1),.O_observed(O_observed),
  .P_begin(tp_begin),.P_learn(tp_learn),.P_template(tp_template),.P_x0(tx0),.P_x1(tx1),.P_y0(ty0),.P_y1(ty1),.P_cx(tcx),.P_cy(tcy),.P_tie_x(ttx),.P_tie_y(tty),
  .P_done(patch_done),.P_quality(patch_quality),.P_dx(patch_dx),.P_dy(patch_dy),.P_score(patch_score),.P_reference_score(patch_zero));
 wire camera_owns=state==CAMERA_START||state==CAMERA_WAIT;
 wire pm_enable,tm_enable,tm_write;wire [AB-1:0] pm_address;wire [9:0] tm_address;wire [14:0] tm_data,tm_q;
 motion_patch_match #(.COLS(COLS),.ROWS(ROWS),.CONTOUR_GUARD(CONTOUR_GUARD)) matcher(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_begin(camera_owns?cp_begin:tp_begin),.I_abort(abort_work),.I_object(!camera_owns),.I_learn(camera_owns?cp_learn:tp_learn),
  .I_template(camera_owns?cp_template:tp_template),.I_x0(camera_owns?cp_x:tx0),.I_x1(camera_owns?cp_x:tx1),.I_y0(camera_owns?cp_y:ty0),.I_y1(camera_owns?cp_y:ty1),
  .I_cx(camera_owns?8'sd0:tcx),.I_cy(camera_owns?8'sd0:tcy),.I_tie_x(camera_owns?8'sd0:ttx),.I_tie_y(camera_owns?8'sd0:tty),
  .O_busy(),.O_done(patch_done),.O_quality(patch_quality),.O_dx(patch_dx),.O_dy(patch_dy),.O_score(patch_score),.O_zero(patch_zero),
  .M_enable(pm_enable),.M_address(pm_address),.M_q(wm_q),.T_enable(tm_enable),.T_write(tm_write),.T_address(tm_address),.T_data(tm_data),.T_q(tm_q));
 motion_object_ram #(.WIDTH(15),.WORDS(1024),.AB(10)) templates(
  .I_clk(I_clk),.A_enable(tm_enable),.A_write(tm_write),.A_address(tm_address),.A_data(tm_data),.A_q(tm_q),
  .B_enable(1'b0),.B_write(1'b0),.B_address(10'd0),.B_data(15'd0),.B_q());
 assign wm_enable=region_busy?rm_enable:pm_enable;
 assign wm_write=region_busy&&rm_write;
 assign wm_address=region_busy?rm_address:pm_address;assign wm_data=rm_data;
 assign O_count=O_ready?count:0;assign O_moving_count=O_ready?moving_count:0;
 assign O_busy=state!=IDLE;
 reg [CB-1:0] union_index;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  state<=IDLE;reset_wait<=0;frame_enable<=0;frame_assist<=0;work_assist<=0;work_bank<=0;work_frame<=0;
  camera_begin<=0;region_begin<=0;tracks_begin<=0;O_valid<=0;O_ready<=0;O_limited<=0;O_frame<=0;O_bbox_valid<=0;O_bx0<=0;O_bx1<=0;O_by0<=0;O_by1<=0;O_cycles<=0;union_index<=0;
 end else begin
  O_valid<=0;camera_begin<=0;region_begin<=0;tracks_begin<=0;
  if(I_start)begin frame_enable<=I_enable;frame_assist<=I_assist;end
  if(state!=IDLE)O_cycles<=O_cycles+1'b1;
  if(I_clear||deadline)begin state<=IDLE;O_ready<=0;O_bbox_valid<=0;O_limited<=deadline;O_valid<=1;end
  else if(complete)begin
   if(frame_enable)begin work_bank<=complete_bank;work_frame<=complete_frame;work_assist<=frame_assist;O_cycles<=0;
    if(overrun)begin reset_wait<=20;state<=RESET_WAIT;O_ready<=0;O_bbox_valid<=0;O_valid<=1;O_limited<=1;end else state<=CAMERA_START;
   end else state<=IDLE;
  end else case(state)
   IDLE:;
   RESET_WAIT:if(reset_wait==0)state<=CAMERA_START;else reset_wait<=reset_wait-1'b1;
   CAMERA_START:begin camera_begin<=1;state<=CAMERA_WAIT;end
   CAMERA_WAIT:if(camera_done)state<=REGION_START;
   REGION_START:begin region_begin<=1;tracks_begin<=1;state<=WORK_WAIT;end
   WORK_WAIT:if(tracks_done)begin
    if(SINGLE_OUTPUT!=0)begin O_bx0<=track_bx0;O_bx1<=track_bx1;O_by0<=track_by0;O_by1<=track_by1;state<=PUBLISH;end
    else begin O_bx0<=WIDTH-1;O_by0<=HEIGHT-1;O_bx1<=0;O_by1<=0;union_index<=0;state<=UNION;end
   end
   UNION:if(union_index<moving_count)begin
    if(O_mx0[union_index*16+:16]<O_bx0)O_bx0<=O_mx0[union_index*16+:16];
    if(O_mx1[union_index*16+:16]>O_bx1)O_bx1<=O_mx1[union_index*16+:16];
    if(O_my0[union_index*16+:16]<O_by0)O_by0<=O_my0[union_index*16+:16];
    if(O_my1[union_index*16+:16]>O_by1)O_by1<=O_my1[union_index*16+:16];union_index<=union_index+1'b1;
   end else state<=PUBLISH;
   PUBLISH:begin O_valid<=1;O_ready<=1;O_limited<=tracks_limited;O_frame<=tracks_frame;O_bbox_valid<=moving_count!=0;state<=IDLE;end
  endcase
 end
endmodule
