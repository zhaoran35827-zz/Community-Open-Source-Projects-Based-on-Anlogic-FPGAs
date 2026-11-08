// Native camera stream: fixed two-register pixel latency, no output pacing
// queue. Gray-world statistics use neutral pixels only. Gains change at SOF.
module awb #(parameter IMG_HEIGHT=1080,IMG_WIDTH=1920,FIXED_GAIN=0,FIELD_FIX=0)(
 input I_clk,I_rst_n,I_tlast,I_tuser,input [95:0] I_tdata,input I_tvalid,
 output I_tready,output reg O_tlast,O_tuser,output reg [95:0] O_tdata,
 output reg O_tvalid,input O_tready,output reg [95:0] O_raw_data,
 output [19:0] O_gain_r,O_gain_b,output reg [7:0] O_scene_luma
);
 localparam BEATS=IMG_WIDTH/4*IMG_HEIGHT;
 // At most 32400 samples at 1920x1080: 24 bits cover 255*32400.
 // Completed frames have a fixed sample count; neutral count saturates at 64.
 localparam SAMPLE_COUNT=(BEATS+15)/16;
 assign I_tready=O_tready;
 reg pending_sof;reg [18:0] index;reg in_frame;
 wire start=I_tvalid&&(I_tuser||pending_sof);
 wire [18:0] bi=start?19'd0:index;
 wire take=I_tvalid&&(in_frame||start);
 wire sample=take&&bi[3:0]==0;
 wire last=take&&bi==BEATS-1;
 wire [7:0] r=I_tdata[23:16],g=I_tdata[15:8],b=I_tdata[7:0];
 wire [7:0] lo=r<g?(r<b?r:b):(g<b?g:b);
 wire [7:0] hi=r>g?(r>b?r:b):(g>b?g:b);
 wire [7:0] neutral_span=(lo>>2)>12?(lo>>2):8'd12;
 wire green_neutral=(r>b?r-b:b-r)<=((lo>>3)>12?(lo>>3):12) && g>=r && g>=b && {1'b0,g}<={1'b0,lo}+{2'd0,lo[7:1]}+{3'd0,lo[7:2]};
 wire wide_neutral={1'b0,hi}<={1'b0,lo}+{2'd0,lo[7:1]};
 wire neutral=lo>=16&&hi<240&&((FIELD_FIX>=2)?(wide_neutral||green_neutral):(FIELD_FIX!=0?({1'b0,hi}<={1'b0,lo}+{1'b0,neutral_span}):({1'b0,hi}<={lo,1'b0})));
 wire [9:0] lum={2'b0,r}+{1'b0,g,1'b0}+{2'b0,b};
 reg [23:0] sr,sg,sb,sl;reg [6:0] n;
 wire [23:0] nr=(start?24'd0:sr)+((sample&&neutral)?{16'd0,r}:24'd0);
 wire [23:0] ng=(start?24'd0:sg)+((sample&&neutral)?{16'd0,g}:24'd0);
 wire [23:0] nb=(start?24'd0:sb)+((sample&&neutral)?{16'd0,b}:24'd0);
 wire [6:0] nn=(start?7'd0:n)+((sample&&neutral&&(start||n<64))?7'd1:7'd0);
 wire [23:0] ls=(start?24'd0:sl)+(sample?{16'd0,lum[9:2]}:24'd0);
 reg [23:0] snap_r,snap_g,snap_b,snap_l;
 reg neutral_ok;reg [2:0] state;reg launch;
 reg [23:0] numerator,denominator;wire done;wire [29:0] quotient;
 awb_ratio_divider u_ratio(.clk(I_clk),.rst_n(I_rst_n),.start(launch),
 .num(numerator),.den(denominator),.done(done),.quotient(quotient));
 reg [7:0] gain_r,gain_b,next_r,next_b,next_l;reg gains_pending,pending_neutral;reg [7:0] frame_luma,reported_r,reported_b;
 function [7:0] bounded;input [29:0] q;
 begin bounded=FIELD_FIX>=2?(q<48?8'd48:q>128?8'd128:q[7:0]):FIELD_FIX!=0?(q<56?8'd56:q>88?8'd88:q[7:0]):(q<48?8'd48:q>96?8'd96:q[7:0]);end endfunction
 function [7:0] slew;input [7:0] old_value,new_value;input [7:0] max_step;
 reg [7:0] d,step;
 begin
   d=old_value>new_value?old_value-new_value:new_value-old_value;
   step=d>>3;if(step==0&&d!=0)step=1;if(step>max_step)step=max_step;
   slew=old_value>new_value?old_value-step:old_value+step;
 end endfunction
 wire [7:0] commit_r=gains_pending&&pending_neutral?slew(gain_r,next_r,8'd2):gain_r;
 wire [7:0] commit_b=gains_pending&&pending_neutral?slew(gain_b,next_b,8'd2):gain_b;
 wire [7:0] active_r=(FIXED_GAIN!=0)?8'd64:(start?commit_r:gain_r);
 wire [7:0] active_b=(FIXED_GAIN!=0)?8'd64:(start?commit_b:gain_b);
 assign O_gain_r=FIXED_GAIN?20'h10000:{2'd0,reported_r,10'd0};
 assign O_gain_b=FIXED_GAIN?20'h10000:{2'd0,reported_b,10'd0};
 reg [15:0] rp[0:3],bp[0:3];reg [95:0] raw1;reg v1,s1,l1;
 integer k;
 function [7:0] clip;input [15:0] v;
 begin clip=(v>=16384)?8'd255:v[13:6];end endfunction
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
   pending_sof<=0;index<=0;in_frame<=0;sr<=0;sg<=0;sb<=0;n<=0;sl<=0;
   snap_r<=0;snap_g<=0;snap_b<=0;snap_l<=0;neutral_ok<=0;
   state<=0;launch<=0;numerator<=0;denominator<=1;
   gain_r<=64;gain_b<=64;next_r<=64;next_b<=64;next_l<=96;O_scene_luma<=96;gains_pending<=0;pending_neutral<=0;frame_luma<=96;reported_r<=64;reported_b<=64;
   raw1<=0;v1<=0;s1<=0;l1<=0;O_tvalid<=0;O_tuser<=0;O_tlast<=0;O_tdata<=0;O_raw_data<=0;
   for(k=0;k<4;k=k+1)begin rp[k]<=0;bp[k]<=0;end
 end else begin
   // Metadata is reported with the output SOF, not the input SOF. The
   // previous frame may still have one pixel beat in the product pipeline.
   if(s1)begin reported_r<=gain_r;reported_b<=gain_b;O_scene_luma<=frame_luma;end
   if(I_tuser)pending_sof<=1;
   if(start)begin
     pending_sof<=0;in_frame<=1;
     if(gains_pending)begin gain_r<=commit_r;gain_b<=commit_b;
       frame_luma<=slew(O_scene_luma,next_l,8'd4);gains_pending<=0;end
   end
   if(take)begin
     index<=bi+1'b1;sr<=nr;sg<=ng;sb<=nb;n<=nn;sl<=ls;
     if(last)begin
       in_frame<=0;
       if(state==0)begin
         snap_r<=nr;snap_g<=ng;snap_b<=nb;snap_l<=ls;neutral_ok<=nn>=64;state<=1;
       end
     end
   end
   launch<=0;
   case(state)
     1:begin numerator<=snap_g;denominator<=snap_r;launch<=1;state<=2;end
     2:if(done)begin snap_r[7:0]<=bounded(quotient);state<=3;end
     3:begin numerator<=snap_g;denominator<=snap_b;launch<=1;state<=4;end
     4:if(done)begin snap_b[7:0]<=bounded(quotient);state<=5;end
     5:begin numerator<=snap_l;denominator<=SAMPLE_COUNT;launch<=1;state<=6;end
     6:if(done)begin
       // Publish one complete bundle. During calculation the pending bundle
       // remains untouched. Reuse consumed snapshot low bytes for work gains.
       next_r<=snap_r[7:0];next_b<=snap_b[7:0];next_l<=quotient[13:6];
       pending_neutral<=neutral_ok;gains_pending<=1;state<=0;
     end
   endcase
   raw1<=I_tdata;v1<=I_tvalid;s1<=start;l1<=I_tlast&&I_tvalid;
   for(k=0;k<4;k=k+1)begin
     rp[k]<=I_tdata[k*24+16+:8]*active_r;
     bp[k]<=I_tdata[k*24+:8]*active_b;
     O_tdata[k*24+:24]<={clip(rp[k]),raw1[k*24+8+:8],clip(bp[k])};
   end
   O_raw_data<=raw1;O_tvalid<=v1;O_tuser<=s1;O_tlast<=l1;
 end
endmodule

// Shared restoring divider: three operations per frame, no line/frame RAM.
// Inputs latched on start; derived from the sequential division in white/isp.
module awb_ratio_divider(input clk,rst_n,start,input [23:0] num,den,
 output reg done,output reg [29:0] quotient);
 reg busy;reg [5:0] count;reg [29:0] q;reg [24:0] rem;reg [23:0] divisor;
 wire [24:0] shifted={rem[23:0],q[29]};wire ge=shifted>={1'b0,divisor};
 wire [29:0] next_q={q[28:0],ge};
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin busy<=0;count<=0;q<=0;rem<=0;divisor<=1;done<=0;quotient<=0;end
 else begin
   done<=0;
   if(start&&!busy)begin
     q<={num,6'd0};rem<=0;divisor<=den;count<=0;
     if(den==0)begin quotient<=0;done<=1;end else busy<=1;
   end else if(busy)begin
     q<=next_q;rem<=ge?shifted-{1'b0,divisor}:shifted;count<=count+1'b1;
     if(count==29)begin quotient<=next_q;busy<=0;done<=1;end
   end
 end
endmodule
