`timescale 1ns/1ps
// One time-shared RGB555 map/template matcher for background and objects.
// Coordinates/displacements are analysis pixels (quarter-grid precision).
// Background matches relative colour patterns; object matches normalised
// foreground fractions at learned silhouette edges, not fixed background RGB.
module motion_patch_match #(parameter COLS=160,ROWS=90,AB=$clog2(COLS*ROWS),CONTOUR_GUARD=0)(
 input I_clk,I_rst_n,I_begin,I_abort,I_object,I_learn,
 input [9:0] I_template,input signed [11:0] I_x0,I_x1,I_y0,I_y1,
 input signed [7:0] I_cx,I_cy,I_tie_x,I_tie_y,
 output reg O_busy,O_done,O_quality,output reg signed [7:0] O_dx,O_dy,
 output reg [15:0] O_score,O_zero,
 output reg M_enable,output reg [AB-1:0] M_address,input [17:0] M_q,
 output reg T_enable,T_write,output reg [9:0] T_address,
 output reg [14:0] T_data,input [14:0] T_q);
 localparam IDLE=0,REF_REQ=1,REF_Q=2,REF_EDGE_REQ=3,REF_EDGE_Q=4,REF_OUT_REQ=5,REF_OUT_Q=6,
  CANDIDATE=7,POINT=8,PIX_REQ=9,PIX_Q=10,PACK=11,BIAS=12,INSIDE_SCORE=13,
  FRAC_CALC=14,FRAC_APPLY=15,NORM_INIT=16,NORM_DIV=17,EVAL=18,NEXT_CAND=19,
  BODY_INIT=20,BODY_PAIR=21,BODY_CHANNEL=22,BODY_NEXT=23,FINISH=24,BG_APPLY=25,FRAC_DIV=26,NORM_MUL=27;
 reg [4:0] state,p;reg object_mode,learn,fine,frac_reference,anchor;
 reg [9:0] base;reg signed [11:0] x0,x1,y0,y1;
 reg signed [7:0] cx,cy,tie_x,tie_y,coarse_x,coarse_y,dx,dy,best_dx,best_dy;
 reg [2:0] kx,ky,edge_index;reg [1:0] tap,score_index;
 // Only four reference interiors and twelve current interior/edge samples.
 // Reference/outside pixels stream from ERAM; outside current samples score
 // immediately. This removes two20-sample random-access register files.
 reg [14:0] reference[0:3],pixel[0:11];reg [11:0] valid;wire [14:0] sample_rgb;
 reg [14:0] v00,v10,v01,v11,bg_reference,last_sample;
 reg [1:0] fx,fy;reg [7:0] gx;reg [6:0] gy;reg point_valid;
 reg [14:0] frac_inside,frac_edge,frac_outside;reg [4:0] cached_fraction;
 reg [3:0] learned_alpha[0:7];reg [7:0] learned_valid;
 reg signed [7:0] bias_r,bias_g,bias_b;
 reg [15:0] score,best_score,zero_score,best_tie;
 reg [6:0] edge_error;reg [3:0] edge_count,horizontal_count,vertical_count,missing;
 reg [11:0] norm_quotient;reg [12:0] norm_coefficient;

 reg [7:0] fraction_remainder;reg [4:0] fraction_denominator;reg [3:0] fraction_quotient;reg fraction_contrast;
 reg invalid_candidate;reg [4:0] valid_count;
 reg body_ok;reg [1:0] body_i,body_j,body_channel;reg [2:0] body_good;
 reg [4:0] body_rm,body_pm,body_rs,body_ps;reg [9:0] body_max_delta;
 reg [4:0] ref_hi_r,ref_hi_g,ref_hi_b,ref_lo_r,ref_lo_g,ref_lo_b;
 function integer mag;input integer v;begin mag=v<0?-v:v;end endfunction
 function [4:0] maxc;input [14:0] c;reg [4:0] m;begin m=c[14:10]>c[9:5]?c[14:10]:c[9:5];maxc=m>c[4:0]?m:c[4:0];end endfunction
 function [4:0] minc;input [14:0] c;reg [4:0] m;begin m=c[14:10]<c[9:5]?c[14:10]:c[9:5];minc=m<c[4:0]?m:c[4:0];end endfunction
 function integer inner_index;input integer i;begin case(i)0:inner_index=0;1:inner_index=2;2:inner_index=1;3:inner_index=3;4:inner_index=0;5:inner_index=1;6:inner_index=2;default:inner_index=3;endcase end endfunction
 function signed [8:0] round4;input signed [9:0] a;reg signed [8:0] q;
 begin q=a>>>2;round4=q+((a[1:0]>2||(a[1:0]==2&&q[0]))?1'b1:1'b0);end endfunction
 // 5-bit channels: horizontal<=124 (7bits), vertical<=496 (9bits).
 // Bound arithmetic before muxing; no32-bit temporary multiplies/adders.
 function [8:0] blend4;input [6:0] a,b;input [1:0] f;reg [8:0] triple;
 begin triple=(f==1?{2'b0,a}:{2'b0,b})*3;
 case(f)0:blend4={a,2'b00};1:blend4=triple+b;2:blend4=({1'b0,a}+{1'b0,b})<<1;3:blend4=a+triple;endcase end endfunction
 function [4:0] bilinear_channel;input [4:0] a,b,c,d;input [1:0] xf,yf;
 reg [6:0] h0,h1;reg [8:0] v;reg [4:0] q;reg [3:0] rem;
 begin h0=blend4({2'b0,a},{2'b0,b},xf);h1=blend4({2'b0,c},{2'b0,d},xf);v=blend4(h0,h1,yf);q=v[8:4];rem=v[3:0];
 bilinear_channel=q+((rem>8||(rem==8&&q[0]))?1'b1:1'b0);end endfunction
 reg signed [12:0] px,py;reg signed [12:0] xa,xb,ya,yb;
 reg [7:0] read_x;reg [6:0] read_y;
 integer i;
 always @* begin
  xa=x0+((x1-x0)>>>2);xb=x0+((((x1-x0)<<<1)+(x1-x0))>>>2);
  ya=y0+((y1-y0)>>>2);yb=y0+((((y1-y0)<<<1)+(y1-y0))>>>2);
  px=0;py=0;
  if(object_mode)case(p)
   0:begin px=xa;py=ya;end 1:begin px=xb;py=ya;end 2:begin px=xa;py=yb;end 3:begin px=xb;py=yb;end
   4:begin px=x0-2;py=ya;end 5:begin px=x0-2;py=yb;end 6:begin px=x1-2;py=ya;end 7:begin px=x1-2;py=yb;end
   8:begin px=xa;py=y0-2;end 9:begin px=xb;py=y0-2;end 10:begin px=xa;py=y1-2;end 11:begin px=xb;py=y1-2;end
   12:begin px=x0-6;py=ya;end 13:begin px=x0-6;py=yb;end 14:begin px=x1+2;py=ya;end 15:begin px=x1+2;py=yb;end
   16:begin px=xa;py=y0-6;end 17:begin px=xb;py=y0-6;end 18:begin px=xa;py=y1+2;end 19:begin px=xb;py=y1+2;end
  endcase
  else begin
   case(p[1:0])0:px=x0-8;1:px=x0-4;2:px=x0+4;3:px=x0+8;endcase
   case(p[3:2])0:py=y0-8;1:py=y0-4;2:py=y0+4;3:py=y0+8;endcase
  end
  read_x=(tap[0]&&gx<COLS-1)?gx+1'b1:gx;
  read_y=(tap[1]&&gy<ROWS-1)?gy+1'b1:gy;
  M_enable=state==PIX_REQ;M_address=COLS==160?(({7'd0,read_y}<<7)+({7'd0,read_y}<<5)+read_x):read_y*COLS+read_x;
 end
 always @* begin
  T_enable=state==REF_REQ||state==REF_EDGE_REQ||state==REF_OUT_REQ||state==PACK&&learn||state==POINT&&!object_mode&&!learn;
  T_write=state==PACK&&learn;T_address=base+p;T_data=sample_rgb;
  if(state==REF_EDGE_REQ)T_address=base+4+edge_index;
  if(state==REF_OUT_REQ)T_address=base+12+edge_index;
 end
 wire [2:0] outside_index=p-12;
 // At most two simultaneous current-cache reads and one reference read.
 // The state machine already serialises all consumers; make that explicit
 // so synthesis can use the same distributed-RAM read ports throughout.
 wire [1:0] ref_read_index = state==REF_EDGE_Q ? inner_index(edge_index) :
   state==PACK ? p[1:0] : score_index;
 wire [3:0] pix_read_a = state==PACK ? inner_index(outside_index) :
   state==INSIDE_SCORE ? {2'b0,score_index} : {2'b0,body_i};
 wire [3:0] pix_read_b = state==PACK ? outside_index+4'd4 : {2'b0,body_j};
 wire [14:0] ref_read = reference[ref_read_index];
 wire [14:0] pix_a = pixel[pix_read_a], pix_b = pixel[pix_read_b];
 wire [3:0] body_shift={body_channel,2'b00}+body_channel;
 wire [4:0] body_rc=pix_a>>body_shift,body_pc=pix_b>>body_shift;
 assign sample_rgb={bilinear_channel(v00[14:10],v10[14:10],v01[14:10],v11[14:10],fx,fy),
  bilinear_channel(v00[9:5],v10[9:5],v01[9:5],v11[9:5],fx,fy),bilinear_channel(v00[4:0],v10[4:0],v01[4:0],v11[4:0],fx,fy)};

 wire signed [5:0] dr=$signed({1'b0,frac_inside[14:10]})-$signed({1'b0,frac_outside[14:10]});
 wire signed [5:0] dg=$signed({1'b0,frac_inside[9:5]})-$signed({1'b0,frac_outside[9:5]});
 wire signed [5:0] db=$signed({1'b0,frac_inside[4:0]})-$signed({1'b0,frac_outside[4:0]});
 wire [5:0] ar=dr<0?-dr:dr,ag=dg<0?-dg:dg,ab=db<0?-db:db;
 wire choose_r=ar>=ag&&ar>=ab,choose_g=!choose_r&&ag>=ab;
 wire signed [5:0] fraction_signed_den=choose_r?dr:choose_g?dg:db;
 wire signed [5:0] fraction_num=choose_r?($signed({1'b0,frac_edge[14:10]})-$signed({1'b0,frac_outside[14:10]})):
  choose_g?($signed({1'b0,frac_edge[9:5]})-$signed({1'b0,frac_outside[9:5]})):($signed({1'b0,frac_edge[4:0]})-$signed({1'b0,frac_outside[4:0]}));
 wire [5:0] fraction_abs_den=fraction_signed_den<0?-fraction_signed_den:fraction_signed_den;
 // Relative contour support: a fixed four-code RGB555 contrast rejected
 // ordinary dim surfaces even when their motion and shape were consistent.
 // Keep the original rule in legacy mode and for bright surfaces. Native
 // dark/mid-tone contours need two/three codes, still above a one-code step.
 wire [4:0] contrast_inside=choose_r?frac_inside[14:10]:choose_g?frac_inside[9:5]:frac_inside[4:0];
 wire [4:0] contrast_outside=choose_r?frac_outside[14:10]:choose_g?frac_outside[9:5]:frac_outside[4:0];
 wire [4:0] contrast_peak=contrast_inside>contrast_outside?contrast_inside:contrast_outside;
 wire contrast_supported=fraction_abs_den>=4 || (CONTOUR_GUARD!=0 &&
   ((contrast_peak<=16 && fraction_abs_den>=2)||(contrast_peak<=24 && fraction_abs_den>=3)));
 wire signed [5:0] fraction_positive_num=fraction_signed_den<0?-fraction_num:fraction_num;
 wire [8:0] twice_fraction_remainder={fraction_remainder,1'b0};
 wire fraction_round=twice_fraction_remainder>{4'b0,fraction_denominator}||
  (twice_fraction_remainder=={4'b0,fraction_denominator}&&fraction_quotient[0]);
 wire reliable=learned_valid[edge_index]&&valid[edge_index+4]&&point_valid&&valid[inner_index(edge_index)];
 wire signed [8:0] tx=$signed(dx)-$signed(tie_x),ty=$signed(dy)-$signed(tie_y);
 // Consumers are mutually exclusive existing states: BODY_CHANNEL,
 // BODY_NEXT, EVAL and NORM_MUL. Sharing does not add a state or cycle.
 wire [8:0] tie_abs_x=tx<0?-tx:tx,tie_abs_y=ty<0?-ty:ty;
 wire [8:0] mult_a0=state==BODY_CHANNEL?{4'd0,body_rc}:state==BODY_NEXT?{4'd0,body_rs}:tie_abs_x;
 wire [8:0] mult_b0=(state==BODY_CHANNEL||state==BODY_NEXT)?{4'd0,body_pm}:tie_abs_x;
 wire [8:0] mult_a1=state==BODY_CHANNEL?{4'd0,body_pc}:state==BODY_NEXT?{4'd0,body_ps}:tie_abs_y;
 wire [8:0] mult_b1=(state==BODY_CHANNEL||state==BODY_NEXT)?{4'd0,body_rm}:tie_abs_y;
 wire [12:0] mult_a2=state==BODY_NEXT?{8'd0,body_rm}:norm_coefficient;
 wire [6:0] mult_b2=state==BODY_NEXT?{2'd0,body_pm}:edge_error;
 wire [17:0] mult_p0=mult_a0*mult_b0,mult_p1=mult_a1*mult_b1;
 wire [19:0] mult_p2=mult_a2*mult_b2;
 wire [9:0] body_cross_a=mult_p0[9:0],body_cross_b=mult_p1[9:0];
 wire [9:0] body_delta=body_cross_a>body_cross_b?body_cross_a-body_cross_b:body_cross_b-body_cross_a;
 wire [9:0] body_product=mult_p2[9:0];
 wire [9:0] body_sat_a=mult_p0[9:0],body_sat_b=mult_p1[9:0];
 wire signed [10:0] body_saturation=$signed({1'b0,body_sat_a})-$signed({1'b0,body_sat_b});
 wire pair_supported=body_max_delta<=(body_product>>2)+4&&mag(body_saturation)<=(body_product>>2)+4;
 wire [20:0] norm_product={1'b0,mult_p2};
 wire [15:0] candidate_tie=mult_p0[15:0]+mult_p1[15:0];

 wire [15:0] total_score=invalid_candidate?16'hffff:object_mode?(edge_count>=4&&horizontal_count>=2&&vertical_count>=2?
   score+{4'b0,norm_quotient}+{missing,5'b0}:16'hffff):(valid_count>=12?score:16'hffff);
 wire improve=total_score<best_score||(total_score==best_score&&candidate_tie<best_tie);

 function [8:0] rgb_cost;input [14:0] a,b;input signed [7:0] br,bg,bb;
 reg signed [8:0] r,g,bl;reg [8:0] ar,ag,ab;
 begin r=$signed({1'b0,a[14:10]})-$signed({1'b0,b[14:10]})-br;
 g=$signed({1'b0,a[9:5]})-$signed({1'b0,b[9:5]})-bg;bl=$signed({1'b0,a[4:0]})-$signed({1'b0,b[4:0]})-bb;
 ar=r<0?-r:r;ag=g<0?-g:g;ab=bl<0?-bl:bl;rgb_cost=ar+ag+ab;end endfunction
 reg signed [9:0] sum_r,sum_g,sum_b;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  sum_r<=0;sum_g<=0;sum_b<=0;
  state<=IDLE;O_busy<=0;O_done<=0;O_quality<=0;O_dx<=0;O_dy<=0;O_score<=0;O_zero<=0;
  object_mode<=0;learn<=0;fine<=0;frac_reference<=0;anchor<=0;base<=0;x0<=0;x1<=0;y0<=0;y1<=0;
  cx<=0;cy<=0;tie_x<=0;tie_y<=0;coarse_x<=0;coarse_y<=0;dx<=0;dy<=0;best_dx<=0;best_dy<=0;
  kx<=0;ky<=0;edge_index<=0;p<=0;tap<=0;score_index<=0;valid<=0;v00<=0;v10<=0;v01<=0;v11<=0;bg_reference<=0;last_sample<=0;
  fx<=0;fy<=0;gx<=0;gy<=0;point_valid<=0;frac_inside<=0;frac_edge<=0;frac_outside<=0;cached_fraction<=0;
  learned_valid<=0;bias_r<=0;bias_g<=0;bias_b<=0;score<=0;best_score<=16'hffff;zero_score<=16'hffff;best_tie<=16'hffff;
  edge_error<=0;edge_count<=0;horizontal_count<=0;vertical_count<=0;missing<=0;
  norm_quotient<=0;norm_coefficient<=0;fraction_remainder<=0;fraction_denominator<=0;fraction_quotient<=0;fraction_contrast<=0;invalid_candidate<=0;valid_count<=0;
  body_ok<=0;body_i<=0;body_j<=0;body_channel<=0;body_good<=0;body_rm<=0;body_pm<=0;body_rs<=0;body_ps<=0;body_max_delta<=0;
  ref_hi_r<=0;ref_hi_g<=0;ref_hi_b<=0;ref_lo_r<=31;ref_lo_g<=31;ref_lo_b<=31;
 end else begin
  O_done<=0;
  if(I_abort)begin state<=IDLE;O_busy<=0;end
  else case(state)
   IDLE:if(I_begin)begin
    O_busy<=1;object_mode<=I_object;learn<=I_learn;base<=I_template;
    x0<=I_x0;x1<=I_x1;y0<=I_y0;y1<=I_y1;cx<=I_cx;cy<=I_cy;tie_x<=I_tie_x;tie_y<=I_tie_y;
    p<=0;kx<=0;ky<=0;fine<=0;anchor<=0;best_score<=16'hffff;best_tie<=16'hffff;zero_score<=16'hffff;learned_valid<=0;best_dx<=0;best_dy<=0;body_ok<=!I_learn;
    ref_hi_r<=0;ref_hi_g<=0;ref_hi_b<=0;ref_lo_r<=31;ref_lo_g<=31;ref_lo_b<=31;valid<=0;valid_count<=0;
    edge_count<=0;horizontal_count<=0;vertical_count<=0;
    if(I_learn)begin dx<=0;dy<=0;state<=POINT;end else state<=I_object?REF_REQ:CANDIDATE;
   end
   REF_REQ:state<=REF_Q;
   REF_Q:begin reference[p[1:0]]<=T_q;
    if(p==3)begin edge_index<=0;state<=REF_EDGE_REQ;end else begin p<=p+1'b1;state<=REF_REQ;end
   end
   REF_EDGE_REQ:state<=REF_EDGE_Q;
   REF_EDGE_Q:begin frac_inside<=ref_read;frac_edge<=T_q;state<=REF_OUT_REQ;end
   REF_OUT_REQ:state<=REF_OUT_Q;
   REF_OUT_Q:begin frac_outside<=T_q;frac_reference<=1;state<=FRAC_CALC;end
   FRAC_CALC:begin
    fraction_denominator<=fraction_abs_den[4:0];fraction_contrast<=contrast_supported;fraction_quotient<=0;
    if(fraction_abs_den==0||fraction_positive_num<=0)begin cached_fraction<={contrast_supported,4'd0};state<=FRAC_APPLY;end
    else if({1'b0,fraction_positive_num}>=fraction_abs_den)begin cached_fraction<={contrast_supported,4'd8};state<=FRAC_APPLY;end
    else begin fraction_remainder<={fraction_positive_num[4:0],3'b0};state<=FRAC_DIV;end
   end
   FRAC_DIV:if(fraction_remainder>={3'b0,fraction_denominator})begin fraction_remainder<=fraction_remainder-fraction_denominator;fraction_quotient<=fraction_quotient+1'b1;end
   else begin cached_fraction<={fraction_contrast,(fraction_quotient+(fraction_round?4'd1:4'd0))};state<=FRAC_APPLY;end
   FRAC_APPLY:if(frac_reference)begin
    // An irregular silhouette need not cross the exact bounding-box edge
    // sample. Its valid foreground/background contrast can have fraction0
    // or8 as well; matching compares the learned fraction, not a rectangle.
    learned_alpha[edge_index]<=cached_fraction[3:0];learned_valid[edge_index]<=cached_fraction[4]&&(CONTOUR_GUARD!=0||(cached_fraction[3:0]>=1&&cached_fraction[3:0]<=7));
    if(edge_index==7)begin anchor<=CONTOUR_GUARD!=0;state<=CANDIDATE;end else begin edge_index<=edge_index+1'b1;state<=REF_EDGE_REQ;end
   end else if(learn)begin
    // Reuse the existing fraction unit while teaching a template. A body
    // with no usable contour cannot ever pass this same matcher's search;
    // do not let such background regions occupy provisional object slots.
    if(cached_fraction[4]&&
       valid[edge_index+4]&&point_valid&&valid[inner_index(edge_index)])begin
     edge_count<=edge_count+1'b1;
     if(edge_index<4)horizontal_count<=horizontal_count+1'b1;else vertical_count<=vertical_count+1'b1;
    end
    if(p==19)state<=BODY_INIT;else begin p<=p+1'b1;state<=POINT;end
   end else begin
    if(reliable)begin
     edge_error<=edge_error+mag($signed({1'b0,cached_fraction[3:0]})-$signed({1'b0,learned_alpha[edge_index]}));
     edge_count<=edge_count+1'b1;if(edge_index<4)horizontal_count<=horizontal_count+1'b1;else vertical_count<=vertical_count+1'b1;
     if(!cached_fraction[4])missing<=missing+1'b1;
    end
    if(p==19)state<=NORM_INIT;else begin p<=p+1'b1;state<=POINT;end
   end
   CANDIDATE:begin
    sum_r<=0;sum_g<=0;sum_b<=0;
    dx<=anchor?tie_x:fine?coarse_x+$signed({1'b0,kx})-2:cx+($signed({1'b0,kx})-2)*4;
    dy<=anchor?tie_y:fine?coarse_y+$signed({1'b0,ky})-2:cy+($signed({1'b0,ky})-2)*4;
    p<=0;valid<=0;score<=0;edge_error<=0;edge_count<=0;horizontal_count<=0;vertical_count<=0;missing<=0;valid_count<=0;invalid_candidate<=0;state<=POINT;
   end
   POINT:begin
    fx<=(px+dx)&3;fy<=(py+dy)&3;
    gx<=px+dx<0?0:px+dx>(COLS-1)*4?COLS-1:(px+dx)>>>2;
    gy<=py+dy<0?0:py+dy>(ROWS-1)*4?ROWS-1:(py+dy)>>>2;
    point_valid<=px+dx>=0&&px+dx<=(COLS-1)*4&&py+dy>=0&&py+dy<=(ROWS-1)*4;
    tap<=0;state<=PIX_REQ;
   end
   PIX_REQ:begin if(tap==0&&!object_mode&&!learn)bg_reference<=T_q;state<=PIX_Q;end
   PIX_Q:begin
    case(tap)0:v00<=M_q[14:0];1:v10<=M_q[14:0];2:v01<=M_q[14:0];3:v11<=M_q[14:0];endcase
    if(tap==3)state<=PACK;else begin tap<=tap+1'b1;state<=PIX_REQ;end
   end
   PACK:begin
    // Sum the same four differences as the original combinational tree.
    // The fourth nonblocking update completes before the existing BIAS
    // cycle, preserving every map/template request and result cycle.
    if(object_mode&&!learn&&p<4)begin
     sum_r<=sum_r+$signed({1'b0,sample_rgb[14:10]})-$signed({1'b0,ref_read[14:10]});
     sum_g<=sum_g+$signed({1'b0,sample_rgb[9:5]})-$signed({1'b0,ref_read[9:5]});
     sum_b<=sum_b+$signed({1'b0,sample_rgb[4:0]})-$signed({1'b0,ref_read[4:0]});
    end
    last_sample<=sample_rgb;
    if(object_mode&&p<12)begin pixel[p]<=sample_rgb;valid[p]<=point_valid;end
    if(learn)begin
     if(sample_rgb[14:10]>ref_hi_r)ref_hi_r<=sample_rgb[14:10];if(sample_rgb[14:10]<ref_lo_r)ref_lo_r<=sample_rgb[14:10];
     if(sample_rgb[9:5]>ref_hi_g)ref_hi_g<=sample_rgb[9:5];if(sample_rgb[9:5]<ref_lo_g)ref_lo_g<=sample_rgb[9:5];
     if(sample_rgb[4:0]>ref_hi_b)ref_hi_b<=sample_rgb[4:0];if(sample_rgb[4:0]<ref_lo_b)ref_lo_b<=sample_rgb[4:0];
     if(CONTOUR_GUARD!=0&&object_mode&&p>=12)begin
      frac_inside<=pix_a;frac_edge<=pix_b;frac_outside<=sample_rgb;edge_index<=outside_index;frac_reference<=0;state<=FRAC_CALC;
     end else if(p==(object_mode?19:15))state<=object_mode?BODY_INIT:FINISH;else begin p<=p+1'b1;state<=POINT;end
    end else if(!object_mode)state<=BG_APPLY;
    else if(p==3)state<=BIAS;
    else if(p>=12)begin
     frac_inside<=pix_a;frac_edge<=pix_b;frac_outside<=sample_rgb;edge_index<=outside_index;frac_reference<=0;state<=FRAC_CALC;
    end else begin p<=p+1'b1;state<=POINT;end
   end
   BG_APPLY:begin
    if(bg_reference[14:10]>ref_hi_r)ref_hi_r<=bg_reference[14:10];if(bg_reference[14:10]<ref_lo_r)ref_lo_r<=bg_reference[14:10];
    if(bg_reference[9:5]>ref_hi_g)ref_hi_g<=bg_reference[9:5];if(bg_reference[9:5]<ref_lo_g)ref_lo_g<=bg_reference[9:5];
    if(bg_reference[4:0]>ref_hi_b)ref_hi_b<=bg_reference[4:0];if(bg_reference[4:0]<ref_lo_b)ref_lo_b<=bg_reference[4:0];
    if(p==0)begin
     bias_r<=$signed({1'b0,last_sample[14:10]})-$signed({1'b0,bg_reference[14:10]});bias_g<=$signed({1'b0,last_sample[9:5]})-$signed({1'b0,bg_reference[9:5]});bias_b<=$signed({1'b0,last_sample[4:0]})-$signed({1'b0,bg_reference[4:0]});
    end else if(point_valid)score<=score+rgb_cost(last_sample,bg_reference,bias_r,bias_g,bias_b);
    if(point_valid)valid_count<=valid_count+1'b1;
    if(p==15)state<=EVAL;else begin p<=p+1'b1;state<=POINT;end
   end
   BIAS:begin
    bias_r<=round4(sum_r);bias_g<=round4(sum_g);bias_b<=round4(sum_b);score_index<=0;state<=INSIDE_SCORE;
    if(mag(round4(sum_r))>3||mag(round4(sum_g))>3||mag(round4(sum_b))>3||valid[3:0]!=4'b1111)begin invalid_candidate<=1;state<=EVAL;end
   end
   INSIDE_SCORE:begin
    score<=score+rgb_cost(pix_a,ref_read,bias_r,bias_g,bias_b);
    if(score_index==3)begin p<=4;state<=POINT;end else score_index<=score_index+1'b1;
   end
   // For err<=64 and count4..8: ceil(16384/count)*err >>9
   // equals floor(32*err/count) exactly (all325 combinations checked).
   NORM_INIT:begin
    case(edge_count)4:norm_coefficient<=4096;5:norm_coefficient<=3277;6:norm_coefficient<=2731;7:norm_coefficient<=2341;8:norm_coefficient<=2048;default:norm_coefficient<=0;endcase
    state<=NORM_MUL;
   end
   NORM_MUL:begin norm_quotient<=norm_product[20:9];state<=EVAL;end
   EVAL:begin
    // In robust object mode O_zero is the explicit prior-motion hypothesis.
    // Before qualification the caller's prior is the camera/background flow.
    // A new object must improve on it, not merely find any low-cost offset.
    if(anchor||(!(CONTOUR_GUARD!=0&&object_mode)&&dx==0&&dy==0))zero_score<=total_score;
    if(improve)begin best_score<=total_score;best_tie<=candidate_tie;best_dx<=dx;best_dy<=dy;end state<=NEXT_CAND;
   end
   NEXT_CAND:if(anchor)begin anchor<=0;kx<=0;ky<=0;state<=CANDIDATE;end
   else if(kx==4&&ky==4)begin
    if(fine)state<=FINISH;else begin fine<=1;coarse_x<=best_dx;coarse_y<=best_dy;kx<=0;ky<=0;state<=CANDIDATE;end
   end else begin if(kx==4)begin kx<=0;ky<=ky+1'b1;end else kx<=kx+1'b1;state<=CANDIDATE;end
   BODY_INIT:begin body_i<=0;body_j<=0;body_good<=0;body_ok<=0;state<=BODY_PAIR;end
   BODY_PAIR:begin
    body_rm<=maxc(pix_a)==0?1:maxc(pix_a);body_pm<=maxc(pix_b)==0?1:maxc(pix_b);
    body_rs<=(maxc(pix_a)==0?1:maxc(pix_a))-minc(pix_a);
    body_ps<=(maxc(pix_b)==0?1:maxc(pix_b))-minc(pix_b);
    body_channel<=0;body_max_delta<=0;state<=BODY_CHANNEL;
   end
   BODY_CHANNEL:begin
    if(body_delta>body_max_delta)body_max_delta<=body_delta;
    if(body_channel==2)state<=BODY_NEXT;else body_channel<=body_channel+1'b1;
   end
   BODY_NEXT:begin
    if(pair_supported)body_good<=body_good+1'b1;
    if(body_j==3)begin
     if(body_good+pair_supported>=3&&valid[3:0]==4'b1111)body_ok<=1;
     body_good<=0;body_j<=0;
     if(body_i==3)begin p<=0;state<=FINISH;end else begin body_i<=body_i+1'b1;state<=BODY_PAIR;end
    end else begin body_j<=body_j+1'b1;state<=BODY_PAIR;end
   end
   FINISH:begin
    O_done<=1;O_busy<=0;state<=IDLE;O_dx<=best_dx;O_dy<=best_dy;O_score<=best_score;O_zero<=zero_score;
    O_quality<=object_mode?(body_ok&&(!learn||CONTOUR_GUARD==0||(edge_count>=4&&horizontal_count>=2&&vertical_count>=2))):({3'b0,ref_hi_r}-{3'b0,ref_lo_r})+({3'b0,ref_hi_g}-{3'b0,ref_lo_g})+({3'b0,ref_hi_b}-{3'b0,ref_lo_b})>=6;
   end
  endcase
 end
endmodule
