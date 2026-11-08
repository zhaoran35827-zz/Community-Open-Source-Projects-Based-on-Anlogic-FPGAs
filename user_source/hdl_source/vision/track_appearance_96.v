`timescale 1ns/1ps
// Eight-point observer. Geometry finishes during row0; every valid sample is
// on row>=1 because the clipped box height is at least8. Require W/4>=2*N+2.
// The main image and observation tap both retain their original timing.
module track_appearance_96 #(parameter IMG_WIDTH=640,IMG_HEIGHT=360,TARGETS=6,LOST_FRAMES=8,RED_PAD=16,COORD_WINDOW=1)(
 input I_clk,I_rst_n,I_clear,I_valid,I_start,input [95:0] I_data,
 input [3:0] I_red_seed,input I_red_assist,input [23:0] I_camera_vectors,
 input [TARGETS-1:0] I_active,I_observed,input [TARGETS*7-1:0] I_tags,
 input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
 output reg O_valid,output reg [TARGETS-1:0] O_visible,output reg [TARGETS*7-1:0] O_tags,
 output [TARGETS*16-1:0] O_xmin,O_xmax,O_ymin,O_ymax,input [7:0] I_frame,output reg [7:0] O_frame,input I_db_ready);
 localparam XB=$clog2(IMG_WIDTH),YB=$clog2(IMG_HEIGHT);
 // The joiner copies at most12 motion words then12 appearance words. Keep
 // last-frame coordinates for32 clocks after a next SOF before preparing new
 // geometry. Native640-wide analysis has160 words/row; points start on row1.
 localparam LIVE_COORDS=COORD_WINDOW!=0&&IMG_WIDTH/4>=32+2*TARGETS+2;
 localparam GEOMETRY_WAIT=LIVE_COORDS?32:0;
 reg [TARGETS*16-1:0] published_x0,published_x1,published_y0,published_y1;
 wire geometry_start;
 generate if(LIVE_COORDS)begin:g_window
  reg [5:0] left;
  always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)left<=0;else if(I_start)left<=32;else if(left!=0)left<=left-1'b1;
  assign geometry_start=left==1;
 end else begin:g_immediate assign geometry_start=I_start;end endgenerate

 reg [XB-1:0] x;reg [YB-1:0] y;reg eof_d;reg [7:0] frame_id;reg frame_assist;reg in_frame;
 reg frame_bad;reg [5:0] geometry_guard;
 wire [XB-1:0] px=I_start?0:x;wire [YB-1:0] py=I_start?0:y;
 wire aborted=I_start&&in_frame;
 wire eof=I_valid&&px==IMG_WIDTH-4&&py==IMG_HEIGHT-1;
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)begin frame_bad<=0;geometry_guard<=0;end
 else if(I_start)begin frame_bad<=!I_db_ready;geometry_guard<=GEOMETRY_WAIT+16;end
 else if(geometry_guard!=0)begin geometry_guard<=geometry_guard-1'b1;if(!I_db_ready)frame_bad<=1;end
 always @(posedge I_clk or negedge I_rst_n)if(!I_rst_n)begin x<=0;y<=0;eof_d<=0;in_frame<=0;frame_id<=0;frame_assist<=0;O_valid<=0;O_frame<=0;end
 else begin
  O_valid<=0;eof_d<=eof;if(I_start)in_frame<=1;else if(eof)in_frame<=0;
  if(I_start)begin x<=0;y<=0;frame_id<=I_frame;frame_assist<=I_red_assist;end
  if(I_valid)begin if(px==IMG_WIDTH-4)begin x<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;end else begin x<=px+4;y<=py;end end
  if(eof_d&&!I_clear&&!frame_bad)begin O_valid<=1;O_frame<=frame_id;end
 end
 wire [TARGETS-1:0] geometry_valid;
 wire [TARGETS*XB-1:0] gx0,gx1,gsx0,gsx1;
 wire [TARGETS*YB-1:0] gy0,gy1,gsy0,gsy1,gsy2,gsy3;
 appearance_geometry_shared #(.W(IMG_WIDTH),.H(IMG_HEIGHT),.N(TARGETS)) geometry(
  .clk(I_clk),.rst(I_rst_n),.start(geometry_start),.active(I_active),.observed(I_observed),.vectors(I_camera_vectors),
  .x0(I_xmin),.x1(I_xmax),.y0(I_ymin),.y1(I_ymax),.valid(geometry_valid),.ox0(gx0),.ox1(gx1),.oy0(gy0),.oy1(gy1),
  .sx0(gsx0),.sx1(gsx1),.sy0(gsy0),.sy1(gsy1),.sy2(gsy2),.sy3(gsy3));
 function [23:0] average4;input [95:0] p;integer c;reg [9:0] q;begin
  for(c=0;c<3;c=c+1)begin q={2'b0,p[c*8+:8]}+{2'b0,p[c*8+24+:8]}+{2'b0,p[c*8+48+:8]}+{2'b0,p[c*8+72+:8]}+2;average4[c*8+:8]=q[9:2];end
 end endfunction
 wire [23:0] mean=average4(I_data);wire [7:0] signature={mean[23:21],mean[15:13],mean[7:6]};
 genvar slot;generate for(slot=0;slot<TARGETS;slot=slot+1)begin:g_slot
  reg frame_active,capture,template_valid,red_known;reg [6:0] frame_tag;
  reg [7:0] score;reg [3:0] samples,red_samples,good,missed;reg [7:0] template[0:7];
  wire active=frame_active&&geometry_valid[slot];
  wire [XB-1:0] fx0=gx0[slot*XB+:XB],fx1=gx1[slot*XB+:XB],sx0=gsx0[slot*XB+:XB],sx1=gsx1[slot*XB+:XB];
  wire [YB-1:0] fy0=gy0[slot*YB+:YB],fy1=gy1[slot*YB+:YB];
  assign O_xmin[slot*16+:16]=LIVE_COORDS?{{(16-XB){1'b0}},fx0}:published_x0[slot*16+:16];
  assign O_xmax[slot*16+:16]=LIVE_COORDS?{{(16-XB){1'b0}},fx1}:published_x1[slot*16+:16];
  assign O_ymin[slot*16+:16]=LIVE_COORDS?{{(16-YB){1'b0}},fy0}:published_y0[slot*16+:16];
  assign O_ymax[slot*16+:16]=LIVE_COORDS?{{(16-YB){1'b0}},fy1}:published_y1[slot*16+:16];
  wire [YB-1:0] sy0=gsy0[slot*YB+:YB],sy1=gsy1[slot*YB+:YB],sy2=gsy2[slot*YB+:YB],sy3=gsy3[slot*YB+:YB];
  wire col0=(px>>2)==(sx0>>2),col1=(px>>2)==(sx1>>2);
  wire row0=py==sy0,row1=py==sy1,row2=py==sy2,row3=py==sy3;
  // Guard row0 while the single geometry engine prepares the current slots.
  wire sample=I_valid&&py!=0&&active&&(col0||col1)&&(row0||row1||row2||row3);
  wire [1:0] sample_row=row0?0:row1?1:row2?2:3;wire [2:0] address={sample_row,col1};
  wire [7:0] old=template[address];
  wire [2:0] dr=signature[7:5]>=old[7:5]?signature[7:5]-old[7:5]:old[7:5]-signature[7:5];
  wire [2:0] dg=signature[4:2]>=old[4:2]?signature[4:2]-old[4:2]:old[4:2]-signature[4:2];
  wire [1:0] db=signature[1:0]>=old[1:0]?signature[1:0]-old[1:0]:old[1:0]-signature[1:0];
  wire [4:0] difference={2'b0,dr}+{2'b0,dg}+{3'b0,db};
  wire observed_good=active&&samples==8&&(capture||(template_valid&&good>=5&&score<=32));
  wire red_support=active&&frame_assist&&red_known&&red_samples>=4;
  always @(posedge I_clk)if(sample&&capture&&!I_clear&&!frame_bad)template[address]<=signature;
  always @(posedge I_clk or negedge I_rst_n)
  if(!I_rst_n)begin frame_active<=0;capture<=0;template_valid<=0;red_known<=0;frame_tag<=0;samples<=0;red_samples<=0;score<=0;good<=0;missed<=0;
   O_visible[slot]<=0;O_tags[slot*7+:7]<=0;published_x0[slot*16+:16]<=0;published_x1[slot*16+:16]<=0;published_y0[slot*16+:16]<=0;published_y1[slot*16+:16]<=0;
  end else if(I_clear)begin frame_active<=0;template_valid<=0;red_known<=0;samples<=0;red_samples<=0;score<=0;good<=0;missed<=0;O_visible[slot]<=0;end
  else begin
   // Publish the preceding frame before a coincident next SOF resets samples.
   if(eof_d&&!frame_bad)begin
    O_visible[slot]<=active&&(observed_good||red_support||(template_valid&&missed<LOST_FRAMES-1));
    O_tags[slot*7+:7]<=frame_tag;published_x0[slot*16+:16]<=fx0;published_x1[slot*16+:16]<=fx1;published_y0[slot*16+:16]<=fy0;published_y1[slot*16+:16]<=fy1;
    if(observed_good||red_support)missed<=0;else if(missed<LOST_FRAMES)missed<=missed+1'b1;
    if(active&&capture&&samples==8)begin template_valid<=1;red_known<=red_samples>=4;end
   end
   if(I_start)begin
    frame_active<=I_active[slot];
    capture<=I_observed[slot]||!(template_valid||(eof_d&&active&&capture&&samples==8))||frame_tag!=I_tags[slot*7+:7]||(aborted&&capture&&samples!=8);
    if(aborted&&capture&&samples!=8)template_valid<=0;
    frame_tag<=I_tags[slot*7+:7];samples<=0;red_samples<=0;score<=0;good<=0;
    if(!I_active[slot]||frame_tag!=I_tags[slot*7+:7])begin template_valid<=0;red_known<=0;missed<=0;end
   end else if(frame_bad)begin frame_active<=0;template_valid<=0;red_known<=0;samples<=0;red_samples<=0;score<=0;good<=0;missed<=0;O_visible[slot]<=0;end
   else if(sample)begin if(samples<8)samples<=samples+1'b1;if(I_red_seed!=0&&red_samples<8)red_samples<=red_samples+1'b1;if(!capture)begin score<=score+difference;if(difference<=3)good<=good+1'b1;end end
  end
 end endgenerate
endmodule


module appearance_geometry_shared #(parameter W=640,H=360,N=6,XB=$clog2(W),YB=$clog2(H))(
 input clk,rst,start,input [N-1:0] active,observed,input [23:0] vectors,
 input [N*16-1:0] x0,x1,y0,y1,
 output reg [N-1:0] valid,
 output reg [N*XB-1:0] ox0,ox1,sx0,sx1,
 output reg [N*YB-1:0] oy0,oy1,sy0,sy1,sy2,sy3);

 reg [N-1:0] act,obs;reg [23:0] cam;reg busy,phase;reg [$clog2(N)-1:0] slot;
 wire [XB-1:0] a=x0[slot*16+:XB],b=x1[slot*16+:XB];
 wire [YB-1:0] c=y0[slot*16+:YB],d=y1[slot*16+:YB];
 wire [XB:0] cx={1'b0,a}+{1'b0,b};wire [YB:0] cy={1'b0,c}+{1'b0,d};
 wire [1:0] region={cy>=H,cx>=W};
 wire signed [2:0] dx=cam[region*6+:3],dy=cam[region*6+3+:3];
 wire signed [XB+3:0] shiftx=obs[slot]?0:-($signed({{(XB+1){dx[2]}},dx})<<<2);
 wire signed [YB+3:0] shifty=obs[slot]?0:-($signed({{(YB+1){dy[2]}},dy})<<<2);
 function [XB-1:0] clipx;input signed [XB+3:0] p;begin clipx=p<0?0:p>=W?W-1:p[XB-1:0];end endfunction
 function [YB-1:0] clipy;input signed [YB+3:0] p;begin clipy=p<0?0:p>=H?H-1:p[YB-1:0];end endfunction
 wire [XB-1:0] ca=clipx($signed({4'd0,a})+shiftx),cb=clipx($signed({4'd0,b})+shiftx);
 wire [YB-1:0] cc=clipy($signed({4'd0,c})+shifty),cd=clipy($signed({4'd0,d})+shifty);
 wire [XB-1:0] left=ox0[slot*XB+:XB],right=ox1[slot*XB+:XB];
 wire [YB-1:0] top=oy0[slot*YB+:YB],bottom=oy1[slot*YB+:YB];
 wire [XB:0] width={1'b0,right}-{1'b0,left}+1'b1;
 wire [YB:0] height={1'b0,bottom}-{1'b0,top}+1'b1;
 wire [XB+2:0] wide_w={2'b0,width};wire [YB+3:0] wide_h={3'b0,height};
 integer k;
 always @(posedge clk or negedge rst)
 if(!rst)begin act<=0;obs<=0;cam<=0;busy<=0;phase<=0;slot<=0;
  valid<=0;ox0<=0;ox1<=0;oy0<=0;oy1<=0;sx0<=0;sx1<=0;sy0<=0;sy1<=0;sy2<=0;sy3<=0;
 end else if(start)begin
  act<=active;
  obs<=observed;cam<=vectors;busy<=1;phase<=0;slot<=0;
 end else if(busy)begin
  if(!phase)begin
   ox0[slot*XB+:XB]<=ca;ox1[slot*XB+:XB]<=cb;oy0[slot*YB+:YB]<=cc;oy1[slot*YB+:YB]<=cd;
   // Clipping cannot enlarge a rectangle, so these two bounds also reject
   // fully off-screen/too-small/unordered rectangles without four wide tests.
   valid[slot]<=act[slot]&&x0[slot*16+:16]<=x1[slot*16+:16]&&x1[slot*16+:16]<W&&y0[slot*16+:16]<=y1[slot*16+:16]&&y1[slot*16+:16]<H&&{1'b0,cb}>={1'b0,ca}+7&&{1'b0,cd}>={1'b0,cc}+7;
   phase<=1;
  end else begin
   sx0[slot*XB+:XB]<=left+(wide_w>>2);sx1[slot*XB+:XB]<=left+(((wide_w<<1)+wide_w)>>2);
   sy0[slot*YB+:YB]<=top+(wide_h>>3);sy1[slot*YB+:YB]<=top+(((wide_h<<1)+wide_h)>>3);
   sy2[slot*YB+:YB]<=top+(((wide_h<<2)+wide_h)>>3);sy3[slot*YB+:YB]<=top+(((wide_h<<2)+(wide_h<<1)+wide_h)>>3);
   phase<=0;if(slot==N-1)busy<=0;else slot<=slot+1'b1;
  end
 end
endmodule

