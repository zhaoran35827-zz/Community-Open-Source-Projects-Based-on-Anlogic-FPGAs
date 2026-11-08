`timescale 1ns/1ps
// A real 2x2 RGB average AFTER Bayer reconstruction. One reused-size RGB row,
// four source pixels/clock -> four analysis pixels every second kept-row beat.
module hd_downsample2_96 #(parameter IMG_WIDTH=1920,IMG_HEIGHT=1080)(
 input I_clk,I_rst_n,I_valid,I_start,I_last,input [95:0] I_data,
 output reg O_valid,O_start,O_last,output reg [95:0] O_data);
 localparam BEATS=IMG_WIDTH/4;
 reg [8:0] x;reg [10:0] y;
 wire [8:0] px=I_start?9'd0:x;wire [10:0] py=I_start?11'd0:y;
 wire [95:0] top;
 visual_row_ram #(.WIDTH(96)) row_mem(.clk(I_clk),.en(I_valid),.we(I_valid),.addr(px),.di(I_data),.dout(top));
 reg v1;reg [8:0] x1;reg [10:0] y1;reg [95:0] bottom;
 reg [47:0] first;
 wire [47:0] means;
 genvar p,c;
 generate for(p=0;p<2;p=p+1)begin:g_pair
 for(c=0;c<3;c=c+1)begin:g_channel
  wire [9:0] sum={2'b0,top[(3-p*2)*24+c*8+:8]}+{2'b0,top[(2-p*2)*24+c*8+:8]}+
                 {2'b0,bottom[(3-p*2)*24+c*8+:8]}+{2'b0,bottom[(2-p*2)*24+c*8+:8]}+10'd2;
  assign means[(1-p)*24+c*8+:8]=sum[9:2];
 end end endgenerate
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin x<=0;y<=0;v1<=0;x1<=0;y1<=0;bottom<=0;first<=0;O_valid<=0;O_start<=0;O_last<=0;O_data<=0;end
 else begin
  if(I_start)begin x<=0;y<=0;end
  if(I_valid)begin
   if(px==BEATS-1)begin x<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;end
   else begin x<=px+1'b1;y<=py;end
  end
  v1<=I_valid&&py[0];
  if(I_valid)begin x1<=px;y1<=py;bottom<=I_data;end
  O_valid<=v1&&x1[0];O_start<=v1&&x1==1&&y1==1;O_last<=v1&&x1==BEATS-1;
  if(v1)begin if(!x1[0])first<=means;else O_data<={first,means};end
 end
endmodule

// Two pixels/clock time-share FOUR multipliers. Input words are at least two
// clocks apart. Apply the SAME frame gains as the full display, retaining raw
// RGB in parallel for motion; raw detection/display disagreement is avoided.
module hd_analysis_color_96(
 input I_clk,I_rst_n,I_valid,I_start,I_last,input [95:0] I_data,
 input [7:0] I_gain_r,I_gain_b,
 output reg O_valid,O_start,O_last,output reg [95:0] O_data,O_raw);
 reg low_job,finish;reg [95:0] held;reg ss,ll,ss2,ll2;
 reg [7:0] gr,gb;reg [15:0] pr0,pb0,pr1,pb1;
 reg [7:0] g0,g1;reg [47:0] high;
 reg [95:0] raw2;
 wire [47:0] pair=I_valid?I_data[95:48]:held[47:0];
 wire [7:0] ar=I_valid?I_gain_r:gr,ab=I_valid?I_gain_b:gb;
 function [7:0] clip;input [15:0] v;begin clip=v>=16384?8'd255:v[13:6];end endfunction
 wire [47:0] corrected={clip(pr0),g0,clip(pb0),clip(pr1),g1,clip(pb1)};
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin low_job<=0;finish<=0;held<=0;ss<=0;ll<=0;ss2<=0;ll2<=0;gr<=64;gb<=64;pr0<=0;pb0<=0;pr1<=0;pb1<=0;g0<=0;g1<=0;high<=0;raw2<=0;O_valid<=0;O_start<=0;O_last<=0;O_data<=0;O_raw<=0;end
 else begin
  low_job<=I_valid;finish<=low_job;
  if(I_valid)begin held<=I_data;ss<=I_start;ll<=I_last;gr<=I_gain_r;gb<=I_gain_b;end
  if(I_valid||low_job)begin
   pr0<=pair[47:40]*ar;pb0<=pair[31:24]*ab;g0<=pair[39:32];
   pr1<=pair[23:16]*ar;pb1<=pair[7:0]*ab;g1<=pair[15:8];
  end
  if(low_job)begin high<=corrected;raw2<=held;ss2<=ss;ll2<=ll;end
  O_valid<=finish;O_start<=finish&&ss2;O_last<=finish&&ll2;
  if(finish)begin O_data<={high,corrected};O_raw<=raw2;end
 end
endmodule
