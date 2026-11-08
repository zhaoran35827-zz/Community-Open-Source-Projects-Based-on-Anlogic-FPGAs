`timescale 1ns/1ps
// Coordinate packets use one36-bit ERAM. Producers hold motion data12 clocks
// and appearance data26 clocks after valid (one12-word motion copy may precede
// its12-word copy). Native camera snapshots are at least one frame apart.
// A new pulse cancels an incomplete copy; work banks cannot be overwritten.
module motion_observation_join #(
 parameter TARGETS=6,CB=3,IMG_WIDTH=640,IMG_HEIGHT=360,MATCH_DISTANCE=64
)(
 input I_clk,I_rst_n,I_clear,
 input I_motion_valid,input [7:0] I_motion_frame,input [CB-1:0] I_motion_count,input [4:0] I_motion_reason,
 input [TARGETS*16-1:0] I_mx0,I_mx1,I_my0,I_my1,
 input I_appearance_valid,input [7:0] I_appearance_frame,input [TARGETS-1:0] I_visible,
 input [TARGETS*7-1:0] I_appearance_tags,input [TARGETS*16-1:0] I_ax0,I_ax1,I_ay0,I_ay1,
 input [TARGETS-1:0] I_track_active,input [TARGETS*7-1:0] I_track_tags,
 output reg O_valid,output reg [7:0] O_frame,output reg [CB-1:0] O_count,
 output reg [TARGETS-1:0] O_observed,output reg O_limited,
 output reg [TARGETS*16-1:0] O_x0,O_x1,O_y0,O_y1,
 input [TARGETS-1:0] I_track_observed,input [TARGETS*16-1:0] I_dbx0,I_dbx1,I_dby0,I_dby1,
 output reg [TARGETS*7-1:0] O_hints,output reg [TARGETS-1:0] O_expired,
 input I_tracker_ready
);
 localparam XB=$clog2(IMG_WIDTH),YB=$clog2(IMG_HEIGHT),IB=$clog2(TARGETS);
 reg mp,ap,busy,mcopy,acopy,mbank,abank,mwrite,awrite,wm,wa;
 reg [7:0] mf,af;reg [CB-1:0] mc,wmc,n;
 reg ml,wlimit;reg [TARGETS-1:0] vis,wvis,used,measured,dbactive,dbobserved;
 reg [TARGETS*7-1:0] tags,wtags,dbtags,hints;
 reg [3:0] mstep,astep,state;reg [IB-1:0] i,j;reg found;
 reg [16:0] best_cost;
 reg [XB-1:0] tx0,tx1,mx0,mx1,bmx0,bmx1;
 reg [YB-1:0] ty0,ty1,my0,my1,bmy0,bmy1;
 reg [TARGETS*16-1:0] bx0,bx1,by0,by1;
 wire [7:0] delta=af-mf;wire signed [7:0] signed_delta=delta;
 wire copying_motion=mcopy;
 wire [3:0] copy_step=copying_motion?mstep:astep;
 wire [2:0] copy_slot=copy_step[3:1];
 wire [TARGETS*16-1:0] copy_x0=copying_motion?I_mx0:I_ax0,copy_x1=copying_motion?I_mx1:I_ax1;
 wire [TARGETS*16-1:0] copy_y0=copying_motion?I_my0:I_ay0,copy_y1=copying_motion?I_my1:I_ay1;
 wire [35:0] write_data=copy_step[0]?
  {{(36-2*YB){1'b0}},copy_y1[copy_slot*16+:YB],copy_y0[copy_slot*16+:YB]}:
  {{(36-2*XB){1'b0}},copy_x1[copy_slot*16+:XB],copy_x0[copy_slot*16+:XB]};
 wire write_enable=(mcopy||acopy)&&!I_clear&&!(copying_motion?I_motion_valid:I_appearance_valid);
 wire [8:0] write_address={3'd0,!copying_motion,copying_motion?mwrite:awrite,copy_slot,copy_step[0]};
 wire appearance_read=state<=1;
 wire read_enable=busy&&(state==0||state==1||state==3||state==4||state==8||state==9);
 wire read_y=state==1||state==4||state==9;
 wire [8:0] read_address={3'd0,appearance_read,appearance_read?wa:wm,appearance_read?{{(3-IB){1'b0}},i}:{{(3-IB){1'b0}},j},read_y};
 wire [35:0] read_data;
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(36),.DATA_WIDTH_B(36),.ADDR_WIDTH_A(9),.ADDR_WIDTH_B(9),
  .DATA_DEPTH_A(512),.DATA_DEPTH_B(512),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
  .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
  .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),.CLKMODE("ASYNC"),
  .INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) packets(
  .doa(),.dia(write_data),.addra(write_address),.cea(write_enable),.ocea(1'b0),.clka(I_clk),.wea(write_enable),.rsta(1'b0),.bea(1'b0),
  .dob(read_data),.dib(36'd0),.addrb(read_address),.ceb(read_enable),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
  .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 wire newborn=wtags[i*7+:7]!=dbtags[i*7+:7]&&dbactive[i]&&dbobserved[i];
 wire eligible=(newborn||wvis[i])&&dbactive[i]&&(newborn||wtags[i*7+:7]==dbtags[i*7+:7]);
 wire [15:0] tx0_full=tx0;
 wire [15:0] tx1_full=tx1;
 wire [15:0] ty0_full=ty0;
 wire [15:0] ty1_full=ty1;
 wire [15:0] mx0_full=mx0;
 wire [15:0] mx1_full=mx1;
 wire [15:0] my0_full=my0;
 wire [15:0] my1_full=my1;
 wire [15:0] bmx0_full=bmx0;
 wire [15:0] bmx1_full=bmx1;
 wire [15:0] bmy0_full=bmy0;
 wire [15:0] bmy1_full=bmy1;
 wire [16:0] asx={1'b0,tx0_full}+{1'b0,tx1_full},asy={1'b0,ty0_full}+{1'b0,ty1_full};
 wire [16:0] msx={1'b0,mx0_full}+{1'b0,mx1_full},msy={1'b0,my0_full}+{1'b0,my1_full};
 wire [15:0] acx=asx[16:1],acy=asy[16:1],mcx=msx[16:1],mcy=msy[16:1];
 wire [15:0] dx=acx>=mcx?acx-mcx:mcx-acx,dy=acy>=mcy?acy-mcy:mcy-acy;
 wire [16:0] cost={1'b0,dx}+{1'b0,dy};
 wire [16:0] oldw={1'b0,tx1_full}-{1'b0,tx0_full},oldh={1'b0,ty1_full}-{1'b0,ty0_full};
 wire [16:0] newsx={1'b0,bmx0_full}+{1'b0,bmx1_full},newsy={1'b0,bmy0_full}+{1'b0,bmy1_full};
 wire [16:0] neww={1'b0,bmx1_full}-{1'b0,bmx0_full},newh={1'b0,bmy1_full}-{1'b0,bmy0_full};
 wire [16:0] width=oldw>neww?oldw:neww,height=oldh>newh?oldh:newh;
 function [15:0] clip_x;input signed [18:0] p;begin clip_x=p<0?0:p>=IMG_WIDTH?IMG_WIDTH-1:{{(16-XB){1'b0}},p[XB-1:0]};end endfunction
 function [15:0] clip_y;input signed [18:0] p;begin clip_y=p<0?0:p>=IMG_HEIGHT?IMG_HEIGHT-1:{{(16-YB){1'b0}},p[YB-1:0]};end endfunction
 wire [15:0] fx0=clip_x($signed({3'b0,newsx[16:1]})-$signed({2'b0,(width>>1)}));
 wire [15:0] fx1=clip_x($signed({3'b0,newsx[16:1]})+$signed({2'b0,width})-$signed({2'b0,(width>>1)}));
 wire [15:0] fy0=clip_y($signed({3'b0,newsy[16:1]})-$signed({2'b0,(height>>1)}));
 wire [15:0] fy1=clip_y($signed({3'b0,newsy[16:1]})+$signed({2'b0,height})-$signed({2'b0,(height>>1)}));
 reg [IB-1:0] best;
 integer k;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  mp<=0;ap<=0;busy<=0;mcopy<=0;acopy<=0;mbank<=0;abank<=0;mwrite<=0;awrite<=0;wm<=0;wa<=0;mstep<=0;astep<=0;
  mf<=0;af<=0;mc<=0;wmc<=0;n<=0;ml<=0;wlimit<=0;vis<=0;wvis<=0;used<=0;measured<=0;dbactive<=0;dbobserved<=0;
  tags<=0;wtags<=0;dbtags<=0;hints<=0;state<=0;i<=0;j<=0;found<=0;best<=0;best_cost<=0;
  tx0<=0;tx1<=0;ty0<=0;ty1<=0;mx0<=0;mx1<=0;my0<=0;my1<=0;bmx0<=0;bmx1<=0;bmy0<=0;bmy1<=0;
  bx0<=0;bx1<=0;by0<=0;by1<=0;O_valid<=0;O_frame<=0;O_count<=0;O_observed<=0;O_limited<=0;O_x0<=0;O_x1<=0;O_y0<=0;O_y1<=0;O_hints<=0;O_expired<=0;
 end else begin
  O_valid<=0;
  if(I_clear)begin mp<=0;ap<=0;busy<=0;mcopy<=0;acopy<=0;O_count<=0;O_observed<=0;O_hints<=0;O_expired<=0;O_limited<=0;end
  else begin
   if(write_enable)begin
    if(copying_motion)begin if(mstep==TARGETS*2-1)begin mcopy<=0;mp<=1;mbank<=mwrite;end else mstep<=mstep+1'b1;end
    else begin if(astep==TARGETS*2-1)begin acopy<=0;ap<=1;abank<=awrite;end else astep<=astep+1'b1;end
   end
   if(!busy&&mp&&ap&&I_tracker_ready)begin
    if(mf!=af)begin if(signed_delta<0)ap<=0;else mp<=0;end
    else begin
     mp<=0;ap<=0;busy<=1;wm<=mbank;wa<=abank;state<=0;i<=0;j<=0;n<=0;used<=0;measured<=0;hints<=0;
     wmc<=mc;wlimit<=ml;O_frame<=mf;wtags<=tags;wvis<=vis;dbactive<=I_track_active;dbtags<=I_track_tags;dbobserved<=I_track_observed;
     for(k=0;k<TARGETS;k=k+1)O_expired[k]<=I_track_active[k]&&tags[k*7+:7]==I_track_tags[k*7+:7]&&!vis[k];
     bx0<=0;bx1<=0;by0<=0;by1<=0;
    end
   end else if(busy)case(state)
    0:state<=1;
    1:begin tx0<=newborn?I_dbx0[i*16+:XB]:read_data[0+:XB];tx1<=newborn?I_dbx1[i*16+:XB]:read_data[XB+:XB];state<=2;end
    2:begin
     ty0<=newborn?I_dby0[i*16+:YB]:read_data[0+:YB];ty1<=newborn?I_dby1[i*16+:YB]:read_data[YB+:YB];
     if(eligible)begin j<=0;found<=0;best_cost<=17'h1ffff;state<=3;end
     else if(i==TARGETS-1)begin j<=0;state<=8;end else begin i<=i+1'b1;state<=0;end
    end
    3:state<=4;
    4:begin mx0<=read_data[0+:XB];mx1<=read_data[XB+:XB];state<=5;end
    5:begin my0<=read_data[0+:YB];my1<=read_data[YB+:YB];state<=6;end
    6:begin
     if(j<wmc&&!used[j]&&cost<=MATCH_DISTANCE&&(!found||cost<best_cost))begin
      found<=1;best<=j;best_cost<=cost;bmx0<=mx0;bmx1<=mx1;bmy0<=my0;bmy1<=my1;
     end
     if(j==TARGETS-1)state<=7;else begin j<=j+1'b1;state<=3;end
    end
    7:begin
     if(n<TARGETS)begin
      for(k=0;k<TARGETS;k=k+1)if(n==k)begin
       bx0[k*16+:16]<=found?fx0:tx0_full;bx1[k*16+:16]<=found?fx1:tx1_full;
       by0[k*16+:16]<=found?fy0:ty0_full;by1[k*16+:16]<=found?fy1:ty1_full;
       measured[k]<=found;hints[k*7+:7]<=newborn?dbtags[i*7+:7]:wtags[i*7+:7];
      end
      n<=n+1'b1;if(found)used[best]<=1;
     end
     if(i==TARGETS-1)begin j<=0;state<=8;end else begin i<=i+1'b1;state<=0;end
    end
    8:state<=9;
    9:begin mx0<=read_data[0+:XB];mx1<=read_data[XB+:XB];state<=10;end
    10:begin my0<=read_data[0+:YB];my1<=read_data[YB+:YB];state<=11;end
    11:begin
     if(j<wmc&&!used[j]&&n<TARGETS)begin
      for(k=0;k<TARGETS;k=k+1)if(n==k)begin bx0[k*16+:16]<=mx0;bx1[k*16+:16]<=mx1;by0[k*16+:16]<=my0;by1[k*16+:16]<=my1;measured[k]<=1;end
      n<=n+1'b1;
     end
     if(j==TARGETS-1)state<=12;else begin j<=j+1'b1;state<=8;end
    end
    12:begin O_count<=n;O_observed<=measured;O_hints<=hints;O_x0<=bx0;O_x1<=bx1;O_y0<=by0;O_y1<=by1;O_limited<=wlimit;O_valid<=1;busy<=0;end
   endcase
   if(I_motion_valid)begin
    mp<=0;mcopy<=1;mstep<=0;mwrite<=busy?~wm:~mbank;
    mf<=I_motion_frame;mc<=I_motion_reason[4:1]!=0?0:(I_motion_count>TARGETS?TARGETS:I_motion_count);ml<=I_motion_reason!=0;
   end
   if(I_appearance_valid)begin ap<=0;acopy<=1;astep<=0;awrite<=busy?~wa:~abank;af<=I_appearance_frame;vis<=I_visible;tags<=I_appearance_tags;end
  end
 end
endmodule
