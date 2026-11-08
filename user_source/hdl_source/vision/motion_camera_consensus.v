`timescale 1ns/1ps
// One serial2-D consensus reducer, reused for global fallback and four local
// affine anchors. A single ambiguous patch cannot create a camera vector.
module motion_camera_consensus #(parameter WIDTH=640,HEIGHT=360,SKIP_FLAT=0)(
 input I_clk,I_rst_n,I_begin,I_abort,I_warm,
 output reg O_busy,O_done,O_warm,
 output reg P_begin,P_learn,output [9:0] P_template,
 output signed [11:0] P_x,P_y,
 input P_done,P_quality,input signed [7:0] P_dx,P_dy,input [15:0] P_score,P_zero,
 output reg [47:0] O_x,O_y,output reg [3:0] O_supported);
 localparam IDLE=0,MATCH=1,WAIT_MATCH=2,LEARN=3,WAIT_LEARN=4,
  GROUP=5,CAND=6,VOTE=7,EVAL=8,NEXT=9,COMMIT=10;
 reg [3:0] state;reg [4:0] patch;
 reg signed [7:0] vx[0:23],vy[0:23];reg [23:0] good,template_good;
 reg [2:0] group;reg [4:0] candidate,sample_index,win_count,votes;
 reg signed [11:0] sum_x,sum_y,win_x,win_y;
 reg signed [7:0] global_x,global_y,win_vx,win_vy;
 reg global_good;reg [8:0] win_norm;
 wire global_group=group==4;
 wire [4:0] ci=global_group?candidate:
  {group[1],3'b000}+{group[1],2'b00}+((candidate>>1)*4)+(group[0]?2:0)+candidate[0];
 wire [4:0] si=global_group?sample_index:
  {group[1],3'b000}+{group[1],2'b00}+((sample_index>>1)*4)+(group[0]?2:0)+sample_index[0];
 wire signed [8:0] dx=$signed(vx[si])-$signed(vx[ci]),dy=$signed(vy[si])-$signed(vy[ci]);
 function [8:0] mag;input signed [8:0] x;begin mag=x<0?-x:x;end endfunction
 wire inlier=good[si]&&mag(dx)<=2&&mag(dy)<=2;
 wire [8:0] norm=mag(vx[ci])+mag(vy[ci]);
 function signed [11:0] average;input signed [11:0] v;input [2:0] c;
 begin case(c)2:average=v*8;3:average=v*16/3;4:average=v*4;5:average=v*16/5;6:average=v*8/3;default:average=0;endcase end endfunction
 assign P_template=patch*16;
 assign P_x=WIDTH*(patch[1:0]*2+1)/8;
 assign P_y=HEIGHT*((patch>>2)*2+1)/12;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  state<=IDLE;O_busy<=0;O_done<=0;O_warm<=0;P_begin<=0;P_learn<=0;patch<=0;good<=0;template_good<=0;
  O_x<=0;O_y<=0;O_supported<=0;group<=0;candidate<=0;sample_index<=0;win_count<=0;votes<=0;
  sum_x<=0;sum_y<=0;win_x<=0;win_y<=0;global_x<=0;global_y<=0;win_vx<=0;win_vy<=0;global_good<=0;win_norm<=0;
 end else begin
  O_done<=0;P_begin<=0;
  if(I_abort)begin state<=IDLE;O_busy<=0;O_warm<=0;template_good<=0;end
  else case(state)
   IDLE:if(I_begin)begin patch<=0;good<=0;O_busy<=1;state<=I_warm?MATCH:LEARN;end
   MATCH:if(SKIP_FLAT!=0&&!template_good[patch])begin
    // The unchanged matcher rejects this template by reference contrast,
    // independently of displacement. Still relearn every frame so a newly
    // textured background is eligible on the following frame.
    good[patch]<=0;vx[patch]<=0;vy[patch]<=0;state<=LEARN;
   end else begin P_begin<=1;P_learn<=0;state<=WAIT_MATCH;end
   WAIT_MATCH:if(P_done)begin
    good[patch]<=P_quality&&P_score<=80;
    vx[patch]<=P_score+16'd2<P_zero?P_dx:8'sd0;vy[patch]<=P_score+16'd2<P_zero?P_dy:8'sd0;state<=LEARN;
   end
   LEARN:begin P_begin<=1;P_learn<=1;state<=WAIT_LEARN;end
   WAIT_LEARN:if(P_done)begin
    template_good[patch]<=P_quality;
    if(patch==23)begin group<=4;state<=GROUP;O_warm<=1;end
    else begin patch<=patch+1'b1;state<=I_warm?MATCH:LEARN;end
   end
   GROUP:begin candidate<=0;win_count<=0;win_norm<=9'h1ff;win_x<=0;win_y<=0;win_vx<=0;win_vy<=0;state<=CAND;end
   CAND:if(!good[ci])state<=NEXT;else begin sample_index<=0;votes<=0;sum_x<=0;sum_y<=0;state<=VOTE;end
   VOTE:begin
    if(inlier)begin votes<=votes+1'b1;sum_x<=sum_x+$signed(vx[si]);sum_y<=sum_y+$signed(vy[si]);end
    if(sample_index==(global_group?23:5))state<=EVAL;else sample_index<=sample_index+1'b1;
   end
   EVAL:begin
    if(votes>win_count||(votes==win_count&&norm<win_norm))begin
     win_count<=votes;win_norm<=norm;win_x<=sum_x;win_y<=sum_y;win_vx<=vx[ci];win_vy<=vy[ci];
    end
    state<=NEXT;
   end
   NEXT:if(candidate==(global_group?23:5))state<=COMMIT;else begin candidate<=candidate+1'b1;state<=CAND;end
   COMMIT:if(global_group)begin global_good<=win_count>=4;global_x<=win_vx;global_y<=win_vy;group<=0;state<=GROUP;end
   else begin
    O_x[group*12+:12]<=win_count>=2?average(win_x,win_count[2:0]):global_good?$signed(global_x)*16:12'sd0;
    O_y[group*12+:12]<=win_count>=2?average(win_y,win_count[2:0]):global_good?$signed(global_y)*16:12'sd0;
    O_supported[group]<=win_count>=2||global_good;
    if(group==3)begin O_done<=1;O_busy<=0;state<=IDLE;end else begin group<=group+1'b1;state<=GROUP;end
   end
  endcase
 end
endmodule
