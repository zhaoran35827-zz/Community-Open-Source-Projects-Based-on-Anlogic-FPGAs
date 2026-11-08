`timescale 1ns/1ps
// Pixel-centre bilinear 3:2 reduction: samples at source .25,1.75 per3.
// Each channel uses (3*a+b+2)>>2 or (a+3*b+2)>>2; shifts/adds only.
// Source4pixels/beat -> display4pixels in2 of every3 source beats on2 of3 rows.
// One RGB line. No frame buffer, clock switch, backpressure or detection tap.
module display_scale_2of3_96 #(parameter IMG_WIDTH=1920,IMG_HEIGHT=1080)(
 input I_clk,I_rst_n,I_valid,I_start,I_last,input [95:0] I_data,
 output reg O_valid,O_start,O_last,output reg [95:0] O_data);
 localparam IN_BEATS=IMG_WIDTH/4,OUT_BEATS=IMG_WIDTH/6;
 reg [8:0] x,ox;reg [10:0] y;reg [1:0] phase,row_phase;
 wire [8:0] px=I_start?9'd0:x;
 wire [10:0] py=I_start?11'd0:y;
 wire [1:0] ph=I_start?2'd0:phase,rph=I_start?2'd0:row_phase;
 reg [95:0] held;
 wire [143:0] source6=ph==1?{held,I_data[95:48]}:{held[47:0],I_data};
 wire [95:0] horizontal;
 genvar n,c;
 generate for(n=0;n<4;n=n+1)begin:g_pixel
  localparam A=(n/2)*3+(n%2),B=A+1;
  for(c=0;c<3;c=c+1)begin:g_channel
   wire [7:0] a=source6[(5-A)*24+c*8+:8],b=source6[(5-B)*24+c*8+:8];
   wire [9:0] total=(n%2==0)?({2'd0,a}+{1'd0,a,1'b0}+{2'd0,b}+10'd2):
                                           ({2'd0,a}+{2'd0,b}+{1'd0,b,1'b0}+10'd2);
   assign horizontal[(3-n)*24+c*8+:8]=total[9:2];
  end
 end endgenerate
 reg hv;reg [95:0] hrgb;reg [8:0] hx;reg [10:0] hy;reg [1:0] hp;
 wire [95:0] previous;
 visual_row_ram #(.WIDTH(96)) row_mem(.clk(I_clk),.en(hv),.we(hv&&hp!=2),.addr(hx),.di(hrgb),.dout(previous));
 reg vv;reg [95:0] bottom;reg [8:0] vx;reg [10:0] vy;reg [1:0] vp;
 wire [95:0] scaled;
 generate for(n=0;n<12;n=n+1)begin:g_vertical
  wire [7:0] a=previous[n*8+:8],b=bottom[n*8+:8];
  wire [9:0] total=vp==1?({2'd0,a}+{1'd0,a,1'b0}+{2'd0,b}+10'd2):
                         ({2'd0,a}+{2'd0,b}+{1'd0,b,1'b0}+10'd2);
  assign scaled[n*8+:8]=total[9:2];
 end endgenerate
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  x<=0;ox<=0;y<=0;phase<=0;row_phase<=0;held<=0;
  hv<=0;hrgb<=0;hx<=0;hy<=0;hp<=0;vv<=0;bottom<=0;vx<=0;vy<=0;vp<=0;
  O_valid<=0;O_start<=0;O_last<=0;O_data<=0;
 end else begin
  hv<=I_valid&&ph!=0;
  if(I_start)begin x<=0;ox<=0;y<=0;phase<=0;row_phase<=0;end
  if(I_valid)begin
   held<=I_data;
   if(ph!=0)begin hrgb<=horizontal;hx<=ox;hy<=py;hp<=rph;ox<=ox+1'b1;end
   if(px==IN_BEATS-1)begin
    x<=0;ox<=0;phase<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;
    row_phase<=py==IMG_HEIGHT-1?0:(rph==2?0:rph+1'b1);
   end else begin x<=px+1'b1;phase<=ph==2?0:ph+1'b1;y<=py;row_phase<=rph;end
  end
  vv<=hv;
  if(hv)begin bottom<=hrgb;vx<=hx;vy<=hy;vp<=hp;end
  O_valid<=vv&&vp!=0;O_start<=vv&&vp!=0&&vx==0&&vy==1;
  O_last<=vv&&vp!=0&&vx==OUT_BEATS-1;
  if(vv&&vp!=0)O_data<=scaled;
 end
endmodule
