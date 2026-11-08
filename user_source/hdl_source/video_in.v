// Frame tags travel with payload through the clock crossing. Never reset a
// live camera FIFO at raw SOF: the ISP may still be delivering the old frame.
module video_in #(parameter FRAME_WORDS=172800, parameter BURST=240, parameter LATENCY_ENABLE=1)(
 input I_rst_n,I_camera_clk,I_camera_frame_start,I_camera_valid,
 input [127:0] I_camera_data,input I_mipi_rx_error,I_ddr_clk,
 input I_display_pause,I_video_out_rd_busy,input [1:0] I_display_rp,
 output O_video_in_wr_busy,output reg [1:0] O_video_out_rp,
 output reg O_frame_valid,output O_ddr_user_wr_en,
 output [24:0] O_ddr_user_addr,output [127:0] O_ddr_user_wr_data,
 input I_ddr_user_ready,
 input [15:0] I_camera_frame_cnt,output reg [15:0] O_video_out_frame_cnt
);
 localparam CW=$clog2(FRAME_WORDS+1);
 reg sof_pending,bad_frame;reg [CW-1:0] camera_count;
 wire sof=I_camera_frame_start||sof_pending;
 wire [CW-1:0] ci=sof?{CW{1'b0}}:camera_count;
 localparam FIFO_BITS=131+((LATENCY_ENABLE!=0)?16:0);
 wire full,empty;wire [8:0] used;wire [FIFO_BITS-1:0] data;
 reg [15:0] frame_tag;
 wire [15:0] incoming_tag=I_camera_frame_start?I_camera_frame_cnt:frame_tag;
 wire [15:0] written_tag;
 generate if(LATENCY_ENABLE!=0)begin:g_tag
   assign written_tag=data[146:131];
 end else begin:g_no_tag
   assign written_tag=16'd0;
 end endgenerate
 always @(posedge I_camera_clk or negedge I_rst_n)
 if(!I_rst_n)begin sof_pending<=0;camera_count<=0;bad_frame<=0;frame_tag<=0;end
 else begin
   if(I_camera_frame_start)begin sof_pending<=1;bad_frame<=0;frame_tag<=I_camera_frame_cnt;end
   if(I_camera_valid)begin
     sof_pending<=0;
     camera_count<=ci==FRAME_WORDS-1?{CW{1'b0}}:ci+1'b1;
     if(full)bad_frame<=1;
   end
 end
 // Frame identity crosses in the SAME FIFO word as SOF and RGB. Stalls,
 // dropped frames and delayed ISP output cannot reorder an independent tag.
 wire [FIFO_BITS-1:0] tagged_data;
 generate if(LATENCY_ENABLE!=0)begin:g_tagged_fifo
   assign tagged_data={incoming_tag,(!sof&&bad_frame),sof,(ci==FRAME_WORDS-1),I_camera_data};
 end else begin:g_untagged_fifo
   assign tagged_data={(!sof&&bad_frame),sof,(ci==FRAME_WORDS-1),I_camera_data};
 end endgenerate
 reg active,pending;reg [7:0] burst_count;reg [9:0] tail_wait;
 // After a dropped/short frame EOF need not be burst aligned. Drain a small
 // tail after a bounded wait rather than stranding it until another frame.
 wire want=(used>=BURST||(used!=0&&tail_wait==1023))&&!I_video_out_rd_busy;
 wire pop=active&&I_ddr_user_ready&&!empty;
 assign O_video_in_wr_busy=active||pending||want;
 soft_fifo_al_4057d6b76aa6 #(.DATA_WIDTH_W(FIFO_BITS),.DATA_WIDTH_R(FIFO_BITS),
 .ADDR_WIDTH_W(9),.ADDR_WIDTH_R(9),.AL_FULL_NUM(480),.AL_EMPTY_NUM(2),
 .SHOW_AHEAD_EN(0),.OUTREG_EN("NOREG")) U_frame_fifo (
 .rst(!I_rst_n),.clkw(I_camera_clk),.we(I_camera_valid&&!full),.di(tagged_data),
 .full_flag(full),.wrusedw(),.afull(),.clkr(I_ddr_clk),.re(pop),.dout(data),
 .valid(),.empty_flag(empty),.rdusedw(used),.aempty());
 function [24:0] base;input [1:0] n;
 begin case(n)0:base=0;1:base=7000000;2:base=14000000;3:base=21000000;endcase end endfunction
 reg [1:0] wp;reg [1:0] free_wp;integer k;reg found;
 // Four buffers: protect both the displayed frame and the latest complete one.
 always @* begin
   free_wp=wp;found=0;
   for(k=1;k<=4;k=k+1)
     if(!found && ((wp+k)&3)!=I_display_rp && ((wp+k)&3)!=O_video_out_rp)
       begin free_wp=(wp+k)&3;found=1;end
 end
 reg collecting;reg [CW-1:0] write_count;reg [24:0] address;
 reg [15:0] write_frame_cnt;
 wire accept_word=pending&&(data[129]||collecting)&&
                         (data[129]||write_count<FRAME_WORDS);
 assign O_ddr_user_wr_en=accept_word;
 assign O_ddr_user_wr_data=data[127:0];
 assign O_ddr_user_addr=data[129]?base(free_wp):address;
 always @(posedge I_ddr_clk or negedge I_rst_n)
 if(!I_rst_n)begin
   active<=0;pending<=0;burst_count<=0;tail_wait<=0;wp<=0;O_video_out_rp<=0;
   O_frame_valid<=0;collecting<=0;write_count<=0;address<=0;
   write_frame_cnt<=0;O_video_out_frame_cnt<=0;
 end else begin
   pending<=pop;
   if(used==0||active)tail_wait<=0;
   else if(tail_wait!=1023)tail_wait<=tail_wait+1'b1;
   if(!active&&!pending&&want)begin active<=1;burst_count<=0;end
   if(pop)begin
     if(burst_count==BURST-1||used<=1)active<=0;
     else burst_count<=burst_count+1'b1;
   end
   if(pending)begin
     if(data[129])begin
       wp<=free_wp;collecting<=1;write_count<=1;address<=base(free_wp)+25'd8;
       write_frame_cnt<=written_tag;
     end else if(accept_word)begin write_count<=write_count+1'b1;address<=address+25'd8;end
     if(data[128])begin
       collecting<=0;
       if(collecting&&!data[129]&&!data[130]&&write_count==FRAME_WORDS-1&&!I_display_pause)begin
         O_video_out_rp<=wp;O_frame_valid<=1;
         O_video_out_frame_cnt<=write_frame_cnt;
       end
     end
   end
 end
endmodule
