module motion_box_confirm #(parameter TARGETS=6,CB=3,MOVE_PIXELS=2,IMG_WIDTH=960,IMG_HEIGHT=540,PREDICT=1,MERGE_GAP=0,MATCH_DISTANCE=128,MIN_CLUSTER_SPAN=0,COORD_BITS=16,DIAGONAL_MERGE=0,COHERENT_MERGE=0,CAMERA_RELATIVE=0,TEMPORAL_VOTES=0,ADAPTIVE_GAP=0,OWNER_GUARD=0,ERAM_CLUSTER=0,FAST_RAW_GEOMETRY=0)(
 input I_clk,I_rst_n,I_clear,I_valid,input [CB-1:0] I_count,
 input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
 output wire O_valid,output wire [CB-1:0] O_count,
 output wire [TARGETS*16-1:0] O_xmin,O_xmax,O_ymin,O_ymax,input [23:0] I_camera_vectors,input [TARGETS-1:0] I_owner_active,input [TARGETS*16-1:0] I_owner_x0,I_owner_x1,I_owner_y0,I_owner_y1);
 wire [CB-1:0] old_count;
 generate if(ERAM_CLUSTER!=0)begin:g_ram
 motion_box_confirm_ram #(.FAST_RAW_GEOMETRY(FAST_RAW_GEOMETRY),.TARGETS(TARGETS),.CB(CB),.MOVE_PIXELS(MOVE_PIXELS),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.PREDICT(PREDICT),.MERGE_GAP(MERGE_GAP),.MATCH_DISTANCE(MATCH_DISTANCE),.MIN_CLUSTER_SPAN(MIN_CLUSTER_SPAN),.COORD_BITS(COORD_BITS),.DIAGONAL_MERGE(DIAGONAL_MERGE),.COHERENT_MERGE(COHERENT_MERGE),.CAMERA_RELATIVE(CAMERA_RELATIVE),.TEMPORAL_VOTES(TEMPORAL_VOTES),.ADAPTIVE_GAP(ADAPTIVE_GAP),.OWNER_GUARD(OWNER_GUARD)) u_core(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(I_clear),.I_valid(I_valid),.I_count(I_count),.I_xmin(I_xmin),.I_xmax(I_xmax),.I_ymin(I_ymin),.I_ymax(I_ymax),.O_valid(O_valid),.O_count(O_count),.O_xmin(O_xmin),.O_xmax(O_xmax),.O_ymin(O_ymin),.O_ymax(O_ymax),.I_camera_vectors(I_camera_vectors),.I_owner_active(I_owner_active),.I_owner_x0(I_owner_x0),.I_owner_x1(I_owner_x1),.I_owner_y0(I_owner_y0),.I_owner_y1(I_owner_y1),.O_history_count(old_count));

 end else begin:g_register
 motion_box_confirm_register #(.TARGETS(TARGETS),.CB(CB),.MOVE_PIXELS(MOVE_PIXELS),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.PREDICT(PREDICT),.MERGE_GAP(MERGE_GAP),.MATCH_DISTANCE(MATCH_DISTANCE),.MIN_CLUSTER_SPAN(MIN_CLUSTER_SPAN),.COORD_BITS(COORD_BITS),.DIAGONAL_MERGE(DIAGONAL_MERGE),.COHERENT_MERGE(COHERENT_MERGE),.CAMERA_RELATIVE(CAMERA_RELATIVE),.TEMPORAL_VOTES(TEMPORAL_VOTES),.ADAPTIVE_GAP(ADAPTIVE_GAP),.OWNER_GUARD(OWNER_GUARD)) u_core(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(I_clear),.I_valid(I_valid),.I_count(I_count),.I_xmin(I_xmin),.I_xmax(I_xmax),.I_ymin(I_ymin),.I_ymax(I_ymax),.O_valid(O_valid),.O_count(O_count),.O_xmin(O_xmin),.O_xmax(O_xmax),.O_ymin(O_ymin),.O_ymax(O_ymax),.I_camera_vectors(I_camera_vectors),.I_owner_active(I_owner_active),.I_owner_x0(I_owner_x0),.I_owner_x1(I_owner_x1),.I_owner_y0(I_owner_y0),.I_owner_y1(I_owner_y1),.O_history_count(old_count));

 end endgenerate
endmodule

`timescale 1ns/1ps
// One sequential nearest-neighbor engine in blanking, six candidates, no frame
// image RAM. Reject stationary flicker boxes, accumulate slow translations.
module motion_box_confirm_register #(parameter TARGETS=6,CB=3,MOVE_PIXELS=2,IMG_WIDTH=960,IMG_HEIGHT=540,PREDICT=1,MERGE_GAP=0,MATCH_DISTANCE=128,MIN_CLUSTER_SPAN=0,COORD_BITS=16,DIAGONAL_MERGE=0,COHERENT_MERGE=0,CAMERA_RELATIVE=0,TEMPORAL_VOTES=0,ADAPTIVE_GAP=0,OWNER_GUARD=0)(
 input I_clk,I_rst_n,I_clear,I_valid,input [CB-1:0] I_count,
 input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
 output reg O_valid,output reg [CB-1:0] O_count,
 output reg [TARGETS*16-1:0] O_xmin,O_xmax,O_ymin,O_ymax,input [23:0] I_camera_vectors,input [TARGETS-1:0] I_owner_active,input [TARGETS*16-1:0] I_owner_x0,I_owner_x1,I_owner_y0,I_owner_y1,output [CB-1:0] O_history_count);
 localparam IB=$clog2(TARGETS);
 reg busy;reg [3:0] state;reg [IB-1:0] i,j,best;
 reg [TARGETS-1:0] owner_valid;reg [IB-1:0] group_owner[0:TARGETS-1];
 reg best_valid;reg [COORD_BITS:0] best_cost;
 reg [CB-1:0] count,old_count,accepted;
 assign O_history_count=old_count;
 reg [TARGETS*16-1:0] x0,x1,y0,y1,wx0,wx1,wy0,wy1;
 reg signed [5:0] group_vx[0:TARGETS-1],group_vy[0:TARGETS-1];
 wire signed [6:0] relative_vx=$signed(group_vx[i])-$signed(group_vx[j]),relative_vy=$signed(group_vy[i])-$signed(group_vy[j]);
 wire coherent=(COHERENT_MERGE==0)||(relative_vx>=-3 && relative_vx<=3 && relative_vy>=-3 && relative_vy<=3);
 reg [COORD_BITS-1:0] old_x[0:TARGETS-1],old_y[0:TARGETS-1],anchor_x[0:TARGETS-1],anchor_y[0:TARGETS-1];
 wire [16:0] owner_xsum={1'b0,I_owner_x0[j*16+:16]}+{1'b0,I_owner_x1[j*16+:16]},owner_ysum={1'b0,I_owner_y0[j*16+:16]}+{1'b0,I_owner_y1[j*16+:16]};
 wire [COORD_BITS-1:0] match_x=state==7?owner_xsum[COORD_BITS:1]:old_x[j],match_y=state==7?owner_ysum[COORD_BITS:1]:old_y[j];
 reg [COORD_BITS-1:0] next_x[0:TARGETS-1],next_y[0:TARGETS-1],next_ax[0:TARGETS-1],next_ay[0:TARGETS-1];
 reg [1:0] hold_frames[0:TARGETS-1],next_hold[0:TARGETS-1];reg [TARGETS-1:0] used;
 reg [23:0] frame_camera,previous_camera;reg [1:0] streak[0:TARGETS-1],next_streak[0:TARGETS-1];
 wire [COORD_BITS:0] xs={1'b0,x0[i*16+:COORD_BITS]}+{1'b0,x1[i*16+:COORD_BITS]},ys={1'b0,y0[i*16+:COORD_BITS]}+{1'b0,y1[i*16+:COORD_BITS]};
 wire [COORD_BITS-1:0] cx=xs[COORD_BITS:1],cy=ys[COORD_BITS:1];
 wire [COORD_BITS-1:0] dx=cx>=match_x?cx-match_x:match_x-cx,dy=cy>=match_y?cy-match_y:match_y-cy;
 wire [COORD_BITS:0] cost={1'b0,dx}+{1'b0,dy};
 wire [COORD_BITS-1:0] ax=cx>=anchor_x[best]?cx-anchor_x[best]:anchor_x[best]-cx,ay=cy>=anchor_y[best]?cy-anchor_y[best]:anchor_y[best]-cy;
 wire [1:0] camera_region={cy>=IMG_HEIGHT/2,cx>=IMG_WIDTH/2};
 wire signed [2:0] camera_dx=frame_camera[camera_region*6+:3],camera_dy=frame_camera[camera_region*6+3+:3];
 wire signed [COORD_BITS+2:0] camera_step_x={{COORD_BITS{camera_dx[2]}},camera_dx}<<<2,camera_step_y={{COORD_BITS{camera_dy[2]}},camera_dy}<<<2;
 wire camera_rebased=frame_camera[camera_region*6+:6]!=previous_camera[camera_region*6+:6];
 wire [COORD_BITS-1:0] base_anchor_x=camera_rebased?old_x[best]:anchor_x[best],base_anchor_y=camera_rebased?old_y[best]:anchor_y[best];
 wire signed [COORD_BITS+2:0] expected_anchor_x=$signed({3'b0,base_anchor_x})-camera_step_x,expected_anchor_y=$signed({3'b0,base_anchor_y})-camera_step_y;
 wire signed [COORD_BITS+3:0] residual_x=$signed({4'b0,cx})-expected_anchor_x,residual_y=$signed({4'b0,cy})-expected_anchor_y;
 wire moved=best_valid && (CAMERA_RELATIVE!=0?(residual_x>=MOVE_PIXELS||residual_x<=-MOVE_PIXELS||residual_y>=MOVE_PIXELS||residual_y<=-MOVE_PIXELS):(ax>=MOVE_PIXELS||ay>=MOVE_PIXELS));
 wire qualified_moved=moved&&(TEMPORAL_VOTES==0||{1'b0,streak[best]}+1>=TEMPORAL_VOTES);
 wire cluster_span_ok=({1'b0,x1[i*16+:16]}-{1'b0,x0[i*16+:16]}+1>=MIN_CLUSTER_SPAN)&&({1'b0,y1[i*16+:16]}-{1'b0,y0[i*16+:16]}+1>=MIN_CLUSTER_SPAN);
 wire accept_box=best_valid&&(qualified_moved||hold_frames[best]!=0);
 wire signed [COORD_BITS:0] velocity_x=$signed({1'b0,cx})-$signed({1'b0,old_x[best]}),velocity_y=$signed({1'b0,cy})-$signed({1'b0,old_y[best]});
 wire signed [COORD_BITS+3:0] world_velocity_x=$signed(velocity_x)+(CAMERA_RELATIVE!=0?camera_step_x:0),world_velocity_y=$signed(velocity_y)+(CAMERA_RELATIVE!=0?camera_step_y:0);
 wire signed [COORD_BITS:0] shift_x=PREDICT==0?17'sd0:velocity_x>16?17'sd16:velocity_x< -16?-17'sd16:velocity_x;
 wire signed [COORD_BITS:0] shift_y=PREDICT==0?17'sd0:velocity_y>16?17'sd16:velocity_y< -16?-17'sd16:velocity_y;
 function [15:0] clip_x;input signed [17:0] q;begin clip_x=q<0?0:q>=IMG_WIDTH?IMG_WIDTH-1:{{(16-COORD_BITS){1'b0}},q[COORD_BITS-1:0]};end endfunction
 function [15:0] clip_y;input signed [17:0] q;begin clip_y=q<0?0:q>=IMG_HEIGHT?IMG_HEIGHT-1:{{(16-COORD_BITS){1'b0}},q[COORD_BITS-1:0]};end endfunction
 // Confirm individual fragments BEFORE merging: a nearby stationary lamp
 // must not borrow a real object's displacement through a merged centroid.
 // Reuse the captured box vectors and index/matching engine in blanking.
 // Merge one pair per scan, compact, then restart for transitive clusters.
 // A projection must overlap: diagonal neighbours never join across a gap.
 wire [COORD_BITS-1:0] ix0=x0[i*16+:16],ix1=x1[i*16+:16],iy0=y0[i*16+:16],iy1=y1[i*16+:16];
 wire [COORD_BITS-1:0] jx0=x0[j*16+:16],jx1=x1[j*16+:16],jy0=y0[j*16+:16],jy1=y1[j*16+:16];
 wire overlap_x=ix0<=jx1 && jx0<=ix1,overlap_y=iy0<=jy1 && jy0<=iy1;
 wire [COORD_BITS:0] iw={1'b0,ix1}-{1'b0,ix0}+1,ih={1'b0,iy1}-{1'b0,iy0}+1,jw={1'b0,jx1}-{1'b0,jx0}+1,jh={1'b0,jy1}-{1'b0,jy0}+1;
 wire [COORD_BITS:0] extent_a=iw>ih?iw:ih,extent_b=jw>jh?jw:jh,extent=extent_a>extent_b?extent_a:extent_b;
 wire [COORD_BITS+1:0] proposed_gap={extent,1'b0};
 wire [7:0] long_gap=ADAPTIVE_GAP!=0?(proposed_gap>96?96:proposed_gap<MERGE_GAP?MERGE_GAP:proposed_gap):MERGE_GAP;
 wire close_x={1'b0,ix0}<={1'b0,jx1}+MERGE_GAP+1 && {1'b0,jx0}<={1'b0,ix1}+MERGE_GAP+1;
 wire close_y={1'b0,iy0}<={1'b0,jy1}+MERGE_GAP+1 && {1'b0,jy0}<={1'b0,iy1}+MERGE_GAP+1;
 wire long_x={1'b0,ix0}<={1'b0,jx1}+long_gap+1 && {1'b0,jx0}<={1'b0,ix1}+long_gap+1;
 wire long_y={1'b0,iy0}<={1'b0,jy1}+long_gap+1 && {1'b0,jy0}<={1'b0,iy1}+long_gap+1;
 wire owner_compatible=OWNER_GUARD==0||!owner_valid[i]||!owner_valid[j]||group_owner[i]==group_owner[j];
 wire merge_pair=coherent&&owner_compatible&&((overlap_x&&long_y)||(overlap_y&&long_x)||(DIAGONAL_MERGE!=0&&close_x&&close_y));
 wire owner_eligible=I_owner_active[j]&&{1'b0,cx}+16>=I_owner_x0[j*16+:16]&&{1'b0,cx}<=({1'b0,I_owner_x1[j*16+:16]}+16)&&
    {1'b0,cy}+16>=I_owner_y0[j*16+:16]&&{1'b0,cy}<=({1'b0,I_owner_y1[j*16+:16]}+16);
 integer n;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  owner_valid<=0;frame_camera<=0;previous_camera<=0;busy<=0;state<=0;i<=0;j<=0;best<=0;best_valid<=0;best_cost<=0;count<=0;old_count<=0;accepted<=0;used<=0;
  x0<=0;x1<=0;y0<=0;y1<=0;wx0<=0;wx1<=0;wy0<=0;wy1<=0;O_valid<=0;O_count<=0;O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;
  for(n=0;n<TARGETS;n=n+1)begin group_owner[n]<=0;streak[n]<=0;next_streak[n]<=0;group_vx[n]<=0;group_vy[n]<=0;old_x[n]<=0;old_y[n]<=0;anchor_x[n]<=0;anchor_y[n]<=0;hold_frames[n]<=0;next_x[n]<=0;next_y[n]<=0;next_ax[n]<=0;next_ay[n]<=0;next_hold[n]<=0;end
 end else begin
  O_valid<=0;
  if(I_clear)begin old_count<=0;busy<=0;O_count<=0;end
  else if(I_valid)begin
   frame_camera<=I_camera_vectors;previous_camera<=frame_camera;owner_valid<=0;
   for(n=0;n<TARGETS;n=n+1)begin
    x0[n*16+:16]<={{(16-COORD_BITS){1'b0}},I_xmin[n*16+:COORD_BITS]};x1[n*16+:16]<={{(16-COORD_BITS){1'b0}},I_xmax[n*16+:COORD_BITS]};
    y0[n*16+:16]<={{(16-COORD_BITS){1'b0}},I_ymin[n*16+:COORD_BITS]};y1[n*16+:16]<={{(16-COORD_BITS){1'b0}},I_ymax[n*16+:COORD_BITS]};
   end
   count<=I_count;
   i<=0;j<=0;best_valid<=0;best_cost<=17'h1ffff;used<=0;accepted<=0;
   wx0<=0;wx1<=0;wy0<=0;wy1<=0;busy<=1;
   state<=I_count==0?2:0;
  end else if(busy)begin
   case(state)
    0:begin
     if(j<old_count&&!used[j]&&cost<=MATCH_DISTANCE&&(!best_valid||cost<best_cost))begin best_valid<=1;best_cost<=cost;best<=j;end
     if(j==TARGETS-1)state<=1;else j<=j+1'b1;
    end
    1:begin
     next_x[i]<=cx;next_y[i]<=cy;next_ax[i]<=best_valid&&!qualified_moved?(CAMERA_RELATIVE!=0?clip_x(expected_anchor_x):anchor_x[best]):cx;next_ay[i]<=best_valid&&!qualified_moved?(CAMERA_RELATIVE!=0?clip_y(expected_anchor_y):anchor_y[best]):cy;
     next_streak[i]<=moved?(streak[best]==3?2'd3:streak[best]+1'b1):0;
     next_hold[i]<=qualified_moved?2:best_valid&&hold_frames[best]!=0?hold_frames[best]-1'b1:0;
     if(best_valid)used[best]<=1;
     if(accept_box)begin
      for(n=0;n<TARGETS;n=n+1)if(accepted==n)begin
       group_vx[n]<=world_velocity_x>16?16:world_velocity_x< -16?-16:world_velocity_x;group_vy[n]<=world_velocity_y>16?16:world_velocity_y< -16?-16:world_velocity_y;
       wx0[n*16+:16]<=clip_x($signed({2'b0,x0[i*16+:16]})+shift_x);wx1[n*16+:16]<=clip_x($signed({2'b0,x1[i*16+:16]})+shift_x);
       wy0[n*16+:16]<=clip_y($signed({2'b0,y0[i*16+:16]})+shift_y);wy1[n*16+:16]<=clip_y($signed({2'b0,y1[i*16+:16]})+shift_y);end
      accepted<=accepted+1'b1;
     end
     if(i==count-1'b1)state<=2;
     else begin i<=i+1'b1;j<=0;best_valid<=0;best_cost<=17'h1ffff;state<=0;end
    end
    2:begin
     old_count<=count;
     for(n=0;n<TARGETS;n=n+1)begin streak[n]<=next_streak[n];old_x[n]<=next_x[n];old_y[n]<=next_y[n];anchor_x[n]<=next_ax[n];anchor_y[n]<=next_ay[n];hold_frames[n]<=next_hold[n];end
     if(MERGE_GAP!=0)begin
      x0<=wx0;x1<=wx1;y0<=wy0;y1<=wy1;count<=accepted;
      wx0<=0;wx1<=0;wy0<=0;wy1<=0;accepted<=0;i<=0;j<=1;
      if(OWNER_GUARD!=0&&I_owner_active!=0&&accepted!=0)begin state<=7;j<=0;best_valid<=0;best_cost<=17'h1ffff;end else state<=accepted>1?3:accepted!=0?5:6;
     end else begin
      O_count<=accepted;O_xmin<=wx0;O_xmax<=wx1;O_ymin<=wy0;O_ymax<=wy1;O_valid<=1;busy<=0;
     end
    end
    3:begin
     if(merge_pair)begin
      if(!owner_valid[i]&&owner_valid[j])begin owner_valid[i]<=1;group_owner[i]<=group_owner[j];end
      for(n=0;n<TARGETS;n=n+1)if(i==n)begin
       x0[n*16+:16]<=ix0<jx0?ix0:jx0;x1[n*16+:16]<=ix1>jx1?ix1:jx1;
       y0[n*16+:16]<=iy0<jy0?iy0:jy0;y1[n*16+:16]<=iy1>jy1?iy1:jy1;
      end
      state<=4;
     end else if(j+1<count)j<=j+1'b1;
     else if(i+2<count)begin i<=i+1'b1;j<=i+2;end
     else begin i<=0;j<=0;state<=5;end
    end
    4:begin
     // i<j, so compaction never overwrites the newly merged rectangle.
     for(n=0;n<TARGETS-1;n=n+1)if(n>=j)begin
      group_vx[n]<=group_vx[n+1];group_vy[n]<=group_vy[n+1];owner_valid[n]<=owner_valid[n+1];group_owner[n]<=group_owner[n+1];
      x0[n*16+:16]<=x0[(n+1)*16+:16];x1[n*16+:16]<=x1[(n+1)*16+:16];
      y0[n*16+:16]<=y0[(n+1)*16+:16];y1[n*16+:16]<=y1[(n+1)*16+:16];
     end
     count<=count-1'b1;i<=0;j<=count>2?1:0;state<=count>2?3:5;
    end
    5:begin
     if(cluster_span_ok)begin
      for(n=0;n<TARGETS;n=n+1)if(accepted==n)begin
       wx0[n*16+:16]<=ix0;wx1[n*16+:16]<=ix1;wy0[n*16+:16]<=iy0;wy1[n*16+:16]<=iy1;
      end
      accepted<=accepted+1'b1;
     end
     if(i==count-1'b1)state<=6;else i<=i+1'b1;
    end
    7:begin
     if(owner_eligible&&(!best_valid||cost<best_cost))begin best_valid<=1;best<=j;best_cost<=cost;end
     if(j==TARGETS-1)state<=8;else j<=j+1'b1;
    end
    8:begin
     for(n=0;n<TARGETS;n=n+1)if(i==n)begin owner_valid[n]<=best_valid;group_owner[n]<=best;end
     if(i==count-1'b1)begin i<=0;j<=1;state<=count>1?3:5;end
     else begin i<=i+1'b1;j<=0;best_valid<=0;best_cost<=17'h1ffff;state<=7;end
    end
    6:begin
     O_count<=accepted;O_xmin<=wx0;O_xmax<=wx1;O_ymin<=wy0;O_ymax<=wy1;O_valid<=1;busy<=0;
    end
   endcase
  end
 end
endmodule

`timescale 1ns/1ps
// Input CCL snapshot stays stable through the short blanking job. A new
// I_valid supersedes the old job. One ERAM holds all accepted/merged boxes.
module motion_box_confirm_ram #(parameter TARGETS=6,CB=3,MOVE_PIXELS=2,IMG_WIDTH=960,IMG_HEIGHT=540,PREDICT=1,MERGE_GAP=0,MATCH_DISTANCE=128,MIN_CLUSTER_SPAN=0,COORD_BITS=16,DIAGONAL_MERGE=0,COHERENT_MERGE=0,CAMERA_RELATIVE=0,TEMPORAL_VOTES=0,ADAPTIVE_GAP=0,OWNER_GUARD=0,FAST_RAW_GEOMETRY=0)(
 input I_clk,I_rst_n,I_clear,I_valid,input [CB-1:0] I_count,
 input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
 output reg O_valid,output reg [CB-1:0] O_count,
 output reg [TARGETS*16-1:0] O_xmin,O_xmax,O_ymin,O_ymax,
 input [23:0] I_camera_vectors,input [TARGETS-1:0] I_owner_active,
 input [TARGETS*16-1:0] I_owner_x0,I_owner_x1,I_owner_y0,I_owner_y1,output [CB-1:0] O_history_count);
 localparam IB=$clog2(TARGETS);
 localparam RAM_WIDTH=2*COORD_BITS>13+IB?2*COORD_BITS:13+IB;
 localparam TS=0,TA=1,TWX=2,TWY=3,TNEXT=4,TFINISH=5,
  RX=6,RY=7,RDONE=8,OSCAN=9,OSAVE=10,MREADJ=11,MEVAL=12,MWX=13,MWY=14,
  CBEGIN=15,CWX=16,CWY=17,CNEXT=18,FBEGIN=19,FCHECK=20,PUBLISH=21,
  HSCOST=22,HAREQ=23,HA_ANC=24,HA_META=25,HSTORE0=26,HSTORE1=27,HSTORE2=28,RMETA=29,TMETA=30,OMETA=31,MMETA=32,CMETA=33;
 reg busy;reg [5:0] state,return_state;reg [IB-1:0] i,j,best,read_slot,compact;
 reg read_j,best_valid;reg [COORD_BITS:0] best_cost;
 reg [CB-1:0] raw_count,old_count,accepted,work_count;
 assign O_history_count=old_count;
 reg [TARGETS-1:0] used;
 reg signed [5:0] ivx,ivy,jvx,jvy;reg iowner_valid,jowner_valid;reg [IB-1:0] iowner,jowner;

 reg history_bank;
 reg [COORD_BITS-1:0] best_x,best_y,old_anchor_x,old_anchor_y;
 reg [1:0] old_hold,old_streak;
 reg [23:0] frame_camera,previous_camera;reg [4:0] hold_output;
 reg [COORD_BITS-1:0] ix0,ix1,iy0,iy1,jx0,jx1,jy0,jy1;
 wire [RAM_WIDTH-1:0] read_data;
 wire raw_mode=state<=TNEXT||state>=HSCOST;
 // The TS history read already precedes HSCOST by a clock. Capture raw
 // endpoints and centre there, rather than recomputing through i and muxes
 // on every history/prediction write. No states or publication clocks added.
 reg [COORD_BITS-1:0] raw_xa,raw_xb,raw_ya,raw_yb,raw_cx,raw_cy;
 wire [COORD_BITS-1:0] input_xa=I_xmin[i*16+:COORD_BITS],input_xb=I_xmax[i*16+:COORD_BITS],input_ya=I_ymin[i*16+:COORD_BITS],input_yb=I_ymax[i*16+:COORD_BITS];
 wire [COORD_BITS:0] input_xsum={1'b0,input_xa}+{1'b0,input_xb},input_ysum={1'b0,input_ya}+{1'b0,input_yb};
 wire [COORD_BITS-1:0] xa=FAST_RAW_GEOMETRY!=0?raw_xa:input_xa,xb=FAST_RAW_GEOMETRY!=0?raw_xb:input_xb,ya=FAST_RAW_GEOMETRY!=0?raw_ya:input_ya,yb=FAST_RAW_GEOMETRY!=0?raw_yb:input_yb;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin raw_xa<=0;raw_xb<=0;raw_ya<=0;raw_yb<=0;raw_cx<=0;raw_cy<=0;end
 else if(FAST_RAW_GEOMETRY!=0 && busy && !I_clear && !I_valid && state==TS)begin
  raw_xa<=input_xa;raw_xb<=input_xb;raw_ya<=input_ya;raw_yb<=input_yb;raw_cx<=input_xsum[COORD_BITS:1];raw_cy<=input_ysum[COORD_BITS:1];
 end
 wire [COORD_BITS:0] xs=raw_mode?({1'b0,xa}+{1'b0,xb}):({1'b0,ix0}+{1'b0,ix1});
 wire [COORD_BITS:0] ys=raw_mode?({1'b0,ya}+{1'b0,yb}):({1'b0,iy0}+{1'b0,iy1});
 wire [COORD_BITS-1:0] cx=(FAST_RAW_GEOMETRY!=0 && raw_mode)?raw_cx:xs[COORD_BITS:1],cy=(FAST_RAW_GEOMETRY!=0 && raw_mode)?raw_cy:ys[COORD_BITS:1];
 wire [16:0] owner_xsum={1'b0,I_owner_x0[j*16+:16]}+{1'b0,I_owner_x1[j*16+:16]},owner_ysum={1'b0,I_owner_y0[j*16+:16]}+{1'b0,I_owner_y1[j*16+:16]};
 wire [COORD_BITS-1:0] match_x=state==OSCAN?owner_xsum[COORD_BITS:1]:read_data[0+:COORD_BITS],match_y=state==OSCAN?owner_ysum[COORD_BITS:1]:read_data[COORD_BITS+:COORD_BITS];
 wire [COORD_BITS-1:0] dx=cx>=match_x?cx-match_x:match_x-cx,dy=cy>=match_y?cy-match_y:match_y-cy;
 wire [COORD_BITS:0] cost={1'b0,dx}+{1'b0,dy};
 wire [COORD_BITS-1:0] ax=cx>=old_anchor_x?cx-old_anchor_x:old_anchor_x-cx,ay=cy>=old_anchor_y?cy-old_anchor_y:old_anchor_y-cy;
 wire [1:0] camera_region={cy>=IMG_HEIGHT/2,cx>=IMG_WIDTH/2};
 wire signed [2:0] camera_dx=frame_camera[camera_region*6+:3],camera_dy=frame_camera[camera_region*6+3+:3];
 wire signed [COORD_BITS+2:0] camera_step_x={{COORD_BITS{camera_dx[2]}},camera_dx}<<<2,camera_step_y={{COORD_BITS{camera_dy[2]}},camera_dy}<<<2;
 wire camera_rebased=frame_camera[camera_region*6+:6]!=previous_camera[camera_region*6+:6];
 wire [COORD_BITS-1:0] base_anchor_x=camera_rebased?best_x:old_anchor_x,base_anchor_y=camera_rebased?best_y:old_anchor_y;
 wire signed [COORD_BITS+2:0] expected_anchor_x=$signed({3'b0,base_anchor_x})-camera_step_x,expected_anchor_y=$signed({3'b0,base_anchor_y})-camera_step_y;
 wire signed [COORD_BITS+3:0] residual_x=$signed({4'b0,cx})-expected_anchor_x,residual_y=$signed({4'b0,cy})-expected_anchor_y;
 wire moved=best_valid&&(CAMERA_RELATIVE!=0?(residual_x>=MOVE_PIXELS||residual_x<=-MOVE_PIXELS||residual_y>=MOVE_PIXELS||residual_y<=-MOVE_PIXELS):(ax>=MOVE_PIXELS||ay>=MOVE_PIXELS));
 wire qualified_moved=moved&&(TEMPORAL_VOTES==0||{1'b0,old_streak}+1>=TEMPORAL_VOTES);
 wire accept_box=best_valid&&(qualified_moved||old_hold!=0);
 wire signed [COORD_BITS:0] velocity_x=$signed({1'b0,cx})-$signed({1'b0,best_x}),velocity_y=$signed({1'b0,cy})-$signed({1'b0,best_y});
 wire signed [COORD_BITS+3:0] world_velocity_x=$signed(velocity_x)+(CAMERA_RELATIVE!=0?camera_step_x:0),world_velocity_y=$signed(velocity_y)+(CAMERA_RELATIVE!=0?camera_step_y:0);
 wire signed [5:0] measured_vx=world_velocity_x>16?16:world_velocity_x< -16?-16:world_velocity_x;
 wire signed [5:0] measured_vy=world_velocity_y>16?16:world_velocity_y< -16?-16:world_velocity_y;
 wire signed [COORD_BITS:0] shift_x=PREDICT==0?17'sd0:velocity_x>16?17'sd16:velocity_x< -16?-17'sd16:velocity_x;
 wire signed [COORD_BITS:0] shift_y=PREDICT==0?17'sd0:velocity_y>16?17'sd16:velocity_y< -16?-17'sd16:velocity_y;
 function [15:0] clip_x;input signed [17:0] q;begin clip_x=q<0?0:q>=IMG_WIDTH?IMG_WIDTH-1:{{(16-COORD_BITS){1'b0}},q[COORD_BITS-1:0]};end endfunction
 function [15:0] clip_y;input signed [17:0] q;begin clip_y=q<0?0:q>=IMG_HEIGHT?IMG_HEIGHT-1:{{(16-COORD_BITS){1'b0}},q[COORD_BITS-1:0]};end endfunction
 wire [15:0] pred_x0=clip_x($signed({{(18-COORD_BITS){1'b0}},xa})+shift_x),pred_x1=clip_x($signed({{(18-COORD_BITS){1'b0}},xb})+shift_x);
 wire [15:0] pred_y0=clip_y($signed({{(18-COORD_BITS){1'b0}},ya})+shift_y),pred_y1=clip_y($signed({{(18-COORD_BITS){1'b0}},yb})+shift_y);
 wire overlap_x=ix0<=jx1&&jx0<=ix1,overlap_y=iy0<=jy1&&jy0<=iy1;
 wire [COORD_BITS:0] iw={1'b0,ix1}-{1'b0,ix0}+1,ih={1'b0,iy1}-{1'b0,iy0}+1,jw={1'b0,jx1}-{1'b0,jx0}+1,jh={1'b0,jy1}-{1'b0,jy0}+1;
 wire [COORD_BITS:0] extent_a=iw>ih?iw:ih,extent_b=jw>jh?jw:jh,extent=extent_a>extent_b?extent_a:extent_b;
 wire [COORD_BITS+1:0] proposed_gap={extent,1'b0};
 wire [7:0] long_gap=ADAPTIVE_GAP!=0?(proposed_gap>96?96:proposed_gap<MERGE_GAP?MERGE_GAP:proposed_gap):MERGE_GAP;
 wire close_x={1'b0,ix0}<={1'b0,jx1}+MERGE_GAP+1&&{1'b0,jx0}<={1'b0,ix1}+MERGE_GAP+1;
 wire close_y={1'b0,iy0}<={1'b0,jy1}+MERGE_GAP+1&&{1'b0,jy0}<={1'b0,iy1}+MERGE_GAP+1;
 wire long_x={1'b0,ix0}<={1'b0,jx1}+long_gap+1&&{1'b0,jx0}<={1'b0,ix1}+long_gap+1;
 wire long_y={1'b0,iy0}<={1'b0,jy1}+long_gap+1&&{1'b0,jy0}<={1'b0,iy1}+long_gap+1;
 wire signed [6:0] relative_vx=$signed(ivx)-$signed(jvx),relative_vy=$signed(ivy)-$signed(jvy);
 wire coherent=COHERENT_MERGE==0||(relative_vx>=-3&&relative_vx<=3&&relative_vy>=-3&&relative_vy<=3);
 wire owner_compatible=OWNER_GUARD==0||!iowner_valid||!jowner_valid||iowner==jowner;
 wire merge_pair=coherent&&owner_compatible&&((overlap_x&&long_y)||(overlap_y&&long_x)||(DIAGONAL_MERGE!=0&&close_x&&close_y));
 wire owner_eligible=I_owner_active[j]&&{1'b0,cx}+16>=I_owner_x0[j*16+:16]&&{1'b0,cx}<=({1'b0,I_owner_x1[j*16+:16]}+16)&&{1'b0,cy}+16>=I_owner_y0[j*16+:16]&&{1'b0,cy}<=({1'b0,I_owner_y1[j*16+:16]}+16);
 wire [COORD_BITS-1:0] minx=ix0<jx0?ix0:jx0,maxx=ix1>jx1?ix1:jx1,miny=iy0<jy0?iy0:jy0,maxy=iy1>jy1?iy1:jy1;
 wire [COORD_BITS-1:0] new_anchor_x=best_valid&&!qualified_moved?(CAMERA_RELATIVE!=0?clip_x(expected_anchor_x):old_anchor_x):cx;
 wire [COORD_BITS-1:0] new_anchor_y=best_valid&&!qualified_moved?(CAMERA_RELATIVE!=0?clip_y(expected_anchor_y):old_anchor_y):cy;
 wire [1:0] new_streak=moved?(old_streak==3?2'd3:old_streak+1'b1):0;
 wire [1:0] new_hold=qualified_moved?2:best_valid&&old_hold!=0?old_hold-1'b1:0;
 wire metadata_write=state==TMETA||state==OMETA||state==MMETA||state==CMETA;
 wire [RAM_WIDTH-1:0] metadata=state==TMETA?{{(RAM_WIDTH-13-IB){1'b0}},1'b0,{IB{1'b0}},measured_vy,measured_vx}:
  state==OMETA?{{(RAM_WIDTH-13-IB){1'b0}},best_valid,best,ivy,ivx}:
  state==MMETA?{{(RAM_WIDTH-13-IB){1'b0}},(iowner_valid||jowner_valid),(iowner_valid?iowner:jowner),ivy,ivx}:
  {{(RAM_WIDTH-13-IB){1'b0}},jowner_valid,jowner,jvy,jvx};
 wire history_write=state==HSTORE0||state==HSTORE1||state==HSTORE2;
 wire [9:0] old_base=history_bank?10'd64:10'd32,new_base=history_bank?10'd32:10'd64;
 wire [9:0] history_read_address={{(6-IB){1'b0}},history_bank,!history_bank,(state==TS?j:best),(state==HAREQ?2'd1:state==HA_ANC?2'd2:2'd0)};
 wire [9:0] history_write_address={{(6-IB){1'b0}},!history_bank,history_bank,i,(state==HSTORE1?2'd1:state==HSTORE2?2'd2:2'd0)};
 wire [9:0] read_address=(state==TS||state==HAREQ||state==HA_ANC)?history_read_address:{{(8-IB){1'b0}},read_slot,(state==RDONE?2'd2:state==RY?2'd1:2'd0)};
 wire read_enable=busy&&!I_clear&&!I_valid&&(state==RX||state==RY||state==RDONE||state==TS||state==HAREQ||state==HA_ANC);
 wire write_en=busy&&!I_clear&&!I_valid&&(state==TWX||state==TWY||state==MWX||state==MWY||state==CWX||state==CWY||history_write||metadata_write);
 wire [IB-1:0] write_slot=(state==TWX||state==TWY||state==TMETA)?accepted[IB-1:0]:(state==MWX||state==MWY||state==OMETA||state==MMETA)?i:compact;
 wire [1:0] write_field=metadata_write?2'd2:(state==TWY||state==MWY||state==CWY)?2'd1:2'd0;
 wire [RAM_WIDTH-1:0] write_data=metadata_write?metadata:state==HSTORE0?{cy,cx}:state==HSTORE1?{new_anchor_y,new_anchor_x}:state==HSTORE2?{{(RAM_WIDTH-4){1'b0}},new_hold,new_streak}:state==TWX?{pred_x1[COORD_BITS-1:0],pred_x0[COORD_BITS-1:0]}:state==TWY?{pred_y1[COORD_BITS-1:0],pred_y0[COORD_BITS-1:0]}:state==MWX?{maxx,minx}:state==MWY?{maxy,miny}:state==CWX?{jx1,jx0}:{jy1,jy0};

 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(RAM_WIDTH),.DATA_WIDTH_B(RAM_WIDTH),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
  .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),.WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
  .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),.ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),.CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) work_boxes(
  .doa(read_data),.dia({RAM_WIDTH{1'b0}}),.addra(read_address),.cea(read_enable),.ocea(1'b0),.clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),
  .dob(),.dib(write_data),.addrb(history_write?history_write_address:{{(8-IB){1'b0}},write_slot,write_field}),.ceb(write_en),.oceb(1'b0),.clkb(I_clk),.web(write_en),.rstb(1'b0),.beb(1'b0),.ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 integer n;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  busy<=0;state<=0;return_state<=0;i<=0;j<=0;best<=0;read_slot<=0;compact<=0;read_j<=0;best_valid<=0;best_cost<=0;raw_count<=0;old_count<=0;accepted<=0;work_count<=0;used<=0;frame_camera<=0;previous_camera<=0;hold_output<=0;
  ix0<=0;ix1<=0;iy0<=0;iy1<=0;jx0<=0;jx1<=0;jy0<=0;jy1<=0;O_valid<=0;O_count<=0;O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;
  history_bank<=0;best_x<=0;best_y<=0;old_anchor_x<=0;old_anchor_y<=0;old_hold<=0;old_streak<=0;
  ivx<=0;ivy<=0;jvx<=0;jvy<=0;iowner_valid<=0;jowner_valid<=0;iowner<=0;jowner<=0;
 end else begin
  O_valid<=0;if(hold_output!=0)hold_output<=hold_output-1'b1;
  if(I_clear)begin old_count<=0;busy<=0;O_count<=0;history_bank<=0;end
  else if(I_valid)begin frame_camera<=I_camera_vectors;previous_camera<=frame_camera;raw_count<=I_count;accepted<=0;used<=0;i<=0;j<=0;best_valid<=0;best_cost<={ (COORD_BITS+1){1'b1}};busy<=1;hold_output<=16;state<=I_count==0?TFINISH:TS;end
  else if(busy)case(state)
   TS:state<=HSCOST;
   HSCOST:begin
    if(j<old_count&&!used[j]&&cost<=MATCH_DISTANCE&&(!best_valid||cost<best_cost))begin
     best_valid<=1;best_cost<=cost;best<=j;best_x<=read_data[0+:COORD_BITS];best_y<=read_data[COORD_BITS+:COORD_BITS];
    end
    if(j==TARGETS-1)state<=HAREQ;else begin j<=j+1'b1;state<=TS;end
   end
   HAREQ:state<=HA_ANC;
   HA_ANC:begin old_anchor_x<=read_data[0+:COORD_BITS];old_anchor_y<=read_data[COORD_BITS+:COORD_BITS];state<=HA_META;end
   HA_META:begin old_hold<=read_data[3:2];old_streak<=read_data[1:0];state<=TA;end
   TA:begin
    if(best_valid)used[best]<=1;
    state<=HSTORE0;
   end
   HSTORE0:state<=HSTORE1;
   HSTORE1:state<=HSTORE2;
   HSTORE2:state<=accept_box?TWX:TNEXT;
   TWX:state<=TWY;
   TWY:state<=TMETA;
   TMETA:begin accepted<=accepted+1'b1;state<=TNEXT;end
   TNEXT:begin if(i==raw_count-1'b1)state<=TFINISH;else begin i<=i+1'b1;j<=0;best_valid<=0;best_cost<={(COORD_BITS+1){1'b1}};state<=TS;end end
   TFINISH:begin
    old_count<=raw_count;history_bank<=!history_bank;
    work_count<=accepted;accepted<=0;i<=0;j<=0;
    if(accepted==0)state<=FBEGIN;
    else if(OWNER_GUARD!=0&&I_owner_active!=0&&MERGE_GAP!=0)begin read_slot<=0;read_j<=0;return_state<=OSCAN;best_valid<=0;best_cost<={(COORD_BITS+1){1'b1}};state<=RX;end
    else if(accepted>1&&MERGE_GAP!=0)begin read_slot<=0;read_j<=0;return_state<=MREADJ;j<=1;state<=RX;end
    else state<=FBEGIN;
   end
   RX:state<=RY;
   RY:begin if(read_j)begin jx0<=read_data[0+:COORD_BITS];jx1<=read_data[COORD_BITS+:COORD_BITS];end else begin ix0<=read_data[0+:COORD_BITS];ix1<=read_data[COORD_BITS+:COORD_BITS];end state<=RDONE;end
   RDONE:begin if(read_j)begin jy0<=read_data[0+:COORD_BITS];jy1<=read_data[COORD_BITS+:COORD_BITS];end else begin iy0<=read_data[0+:COORD_BITS];iy1<=read_data[COORD_BITS+:COORD_BITS];end state<=RMETA;end
   RMETA:begin
    if(read_j)begin jvx<=read_data[5:0];jvy<=read_data[11:6];jowner<=read_data[12+:IB];jowner_valid<=read_data[12+IB];end
    else begin ivx<=read_data[5:0];ivy<=read_data[11:6];iowner<=read_data[12+:IB];iowner_valid<=read_data[12+IB];end
    state<=return_state;
   end
   OSCAN:begin if(owner_eligible&&(!best_valid||cost<best_cost))begin best_valid<=1;best<=j;best_cost<=cost;end if(j==TARGETS-1)state<=OSAVE;else j<=j+1'b1;end
   OSAVE:state<=OMETA;
   OMETA:begin
    if(i==work_count-1'b1)begin i<=0;j<=1;if(work_count>1)begin read_slot<=0;read_j<=0;return_state<=MREADJ;state<=RX;end else state<=FBEGIN;end
    else begin i<=i+1'b1;read_slot<=i+1'b1;read_j<=0;return_state<=OSCAN;j<=0;best_valid<=0;best_cost<={(COORD_BITS+1){1'b1}};state<=RX;end
   end
   MREADJ:begin read_slot<=j;read_j<=1;return_state<=MEVAL;state<=RX;end
   MEVAL:begin
    if(merge_pair)state<=MWX;
    else if(j+1<work_count)begin j<=j+1'b1;state<=MREADJ;end
    else if(i+2<work_count)begin i<=i+1'b1;j<=i+2;read_slot<=i+1'b1;read_j<=0;return_state<=MREADJ;state<=RX;end
    else state<=FBEGIN;
   end
   MWX:state<=MWY;
   MWY:state<=MMETA;
   MMETA:begin compact<=j;state<=CBEGIN;end
   CBEGIN:begin
    if(compact+1<work_count)begin read_slot<=compact+1'b1;read_j<=1;return_state<=CWX;state<=RX;end
    else begin work_count<=work_count-1'b1;i<=0;j<=1;if(work_count>2)begin read_slot<=0;read_j<=0;return_state<=MREADJ;state<=RX;end else state<=FBEGIN;end
   end
   CWX:state<=CWY;
   CWY:state<=CMETA;
   CMETA:begin compact<=compact+1'b1;state<=CBEGIN;end
   FBEGIN:if(hold_output==0)begin
    i<=0;accepted<=0;O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;
    if(work_count==0)state<=PUBLISH;else begin read_slot<=0;read_j<=0;return_state<=FCHECK;state<=RX;end
   end
   FCHECK:begin
    if(MERGE_GAP==0||(iw>=MIN_CLUSTER_SPAN&&ih>=MIN_CLUSTER_SPAN))begin
     for(n=0;n<TARGETS;n=n+1)if(accepted==n)begin O_xmin[n*16+:16]<=ix0;O_xmax[n*16+:16]<=ix1;O_ymin[n*16+:16]<=iy0;O_ymax[n*16+:16]<=iy1;end
     accepted<=accepted+1'b1;
    end
    if(i==work_count-1'b1)state<=PUBLISH;else begin i<=i+1'b1;read_slot<=i+1'b1;read_j<=0;return_state<=FCHECK;state<=RX;end
   end
   PUBLISH:begin O_count<=accepted;O_valid<=1;busy<=0;end
  endcase
 end
endmodule
