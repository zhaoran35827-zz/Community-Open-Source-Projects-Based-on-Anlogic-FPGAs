// Read requests reserve FIFO space before DDR responses arrive. At VSYNC,
// drain all old responses, flush, then prefetch the selected complete frame.
module video_out #(parameter FRAME_WORDS=172800,parameter BURST=240,parameter LATENCY_ENABLE=1)(
 input I_rst_n,I_ddr_clk,output O_video_out_rd_busy,input I_video_in_wr_busy,
 input [1:0] I_video_out_rp,input I_frame_valid,output reg [1:0] O_display_rp,
 output O_ddr_user_rd_en,output reg [24:0] O_ddr_user_addr,
 input I_ddr_user_ready,I_ddr_user_rd_valid,input [127:0] I_ddr_user_rd_data,
 input I_dsi_clk,I_video_vsync,I_video_rd_en,output reg [23:0] O_vdieo_data,
 input [15:0] I_video_out_frame_cnt,output [15:0] O_display_frame_cnt,
 output O_display_frame_valid,O_display_frame_update
);
 reg [2:0] vs;wire sof=vs[1]&&!vs[2];
 wire [16:0] display_metadata;wire can_display;
 generate if(LATENCY_ENABLE!=0)begin:g_latency
   // Snapshot the exact same published frame as O_display_rp below. The
   // mailbox holds its bundled payload across clocks; no binary bus CDC.
   cdc_frame_mailbox #(.WIDTH(17)) u_read_frame_cdc(
     .I_src_clk(I_ddr_clk),.I_src_rst_n(I_rst_n),.I_src_valid(sof),
     .I_src_data({I_frame_valid,I_video_out_frame_cnt}),.O_src_ready(),
     .I_dst_clk(I_dsi_clk),.I_dst_rst_n(I_rst_n),
     .O_dst_valid(O_display_frame_update),.O_dst_data(display_metadata));
 end else begin:g_no_latency
   assign display_metadata=17'd0;
   assign O_display_frame_update=1'b0;
 end endgenerate
 assign O_display_frame_cnt=display_metadata[15:0];
 // A selected buffer is not an image until this frame's prefetch is usable.
 assign O_display_frame_valid=display_metadata[16]&&can_display;
 always @(posedge I_ddr_clk or negedge I_rst_n)
 if(!I_rst_n)vs<=0;else vs<={vs[1:0],I_video_vsync};
 localparam DRAIN=0,FLUSH=1,RUN=2;
 reg [1:0] state;reg [4:0] reset_count;reg selected_valid,active,ready;
 localparam CW=$clog2(FRAME_WORDS+1);
 reg [7:0] burst_count;reg [9:0] outstanding;reg [CW-1:0] requested;
 wire fifo_reset=!I_rst_n||state==FLUSH;
 wire [8:0] used;wire empty,full;wire [127:0] fifo_data;
 wire [10:0] reserved={2'b0,used}+{1'b0,outstanding};
 assign O_video_out_rd_busy=active;
 assign O_ddr_user_rd_en=state==RUN&&!sof&&active&&I_ddr_user_ready;
 function [24:0] base;input [1:0] n;
 begin case(n)0:base=0;1:base=7000000;2:base=14000000;3:base=21000000;endcase end endfunction
 always @(posedge I_ddr_clk or negedge I_rst_n)
 if(!I_rst_n)begin
   state<=DRAIN;reset_count<=0;selected_valid<=0;active<=0;ready<=0;
   burst_count<=0;outstanding<=0;requested<=0;O_ddr_user_addr<=0;O_display_rp<=0;
 end else begin
   case({O_ddr_user_rd_en,I_ddr_user_rd_valid})
     2'b10:outstanding<=outstanding+1'b1;
     2'b01:if(outstanding!=0)outstanding<=outstanding-1'b1;
   endcase
   if(sof)begin
     state<=DRAIN;active<=0;ready<=0;requested<=0;
     O_display_rp<=I_video_out_rp;selected_valid<=I_frame_valid;
   end else case(state)
     DRAIN:if(outstanding==0)begin state<=FLUSH;reset_count<=0;end
     FLUSH:if(reset_count==15)begin state<=RUN;O_ddr_user_addr<=base(O_display_rp);end
           else reset_count<=reset_count+1'b1;
     RUN:begin
       if(used>=64)ready<=selected_valid;
       if(!active&&selected_valid&&!I_video_in_wr_busy&&reserved<=511-BURST-16&&requested<FRAME_WORDS)
         begin active<=1;burst_count<=0;end
       if(O_ddr_user_rd_en)begin
         O_ddr_user_addr<=O_ddr_user_addr+25'd8;requested<=requested+1'b1;
         if(burst_count==BURST-1||requested==FRAME_WORDS-1)active<=0;
         else burst_count<=burst_count+1'b1;
       end
     end
   endcase
 end
 wire pop;
 w128_d512_fifo U_w128_d512_fifo (
 .rst(fifo_reset),.clkw(I_ddr_clk),.we(I_ddr_user_rd_valid&&state==RUN&&!sof&&!full),
 .di(I_ddr_user_rd_data),.wrusedw(used),.afull(),.full_flag(full),
 .clkr(I_dsi_clk),.re(pop),.dout(fifo_data),.rdusedw(),.valid(),.empty_flag(empty),.aempty());
 // Two-word lookahead separates synchronous FIFO latency from RGB byte assembly.
 reg [1:0] ready_sync;reg vs_d,decided,use_frame,poison,seen_not_ready;
 reg [127:0] q0,q1,current;reg [1:0] qcount;reg pending;
 reg [3:0] phase;reg [23:0] pixel;reg pixel_valid;
 wire frame_start=I_video_vsync&&!vs_d;
 wire boundary=phase==0||phase==5||phase==10;
 assign can_display=(decided?use_frame:(ready_sync[1]&&seen_not_ready))&&!poison&&!fifo_reset;
 wire take=I_video_rd_en&&can_display&&boundary&&qcount!=0;
 assign pop=ready_sync[1]&&!fifo_reset&&!empty&&({1'b0,qcount}+pending<2);
 reg [23:0] assembled;
 always @* begin
   assembled=0;
   case(phase)
     0:assembled=q0[127:104];1:assembled=current[103:80];2:assembled=current[79:56];
     3:assembled=current[55:32];4:assembled=current[31:8];
     5:assembled={current[7:0],q0[127:112]};6:assembled=current[111:88];
     7:assembled=current[87:64];8:assembled=current[63:40];9:assembled=current[39:16];
     10:assembled={current[15:0],q0[127:120]};11:assembled=current[119:96];
     12:assembled=current[95:72];13:assembled=current[71:48];
     14:assembled=current[47:24];15:assembled=current[23:0];
   endcase
 end
 always @(posedge I_dsi_clk or negedge I_rst_n)
 if(!I_rst_n)begin
   ready_sync<=0;vs_d<=0;decided<=0;use_frame<=0;poison<=0;seen_not_ready<=0;
   q0<=0;q1<=0;current<=0;qcount<=0;pending<=0;phase<=0;pixel<=0;pixel_valid<=0;O_vdieo_data<=0;
 end else begin
   ready_sync<={ready_sync[0],ready};vs_d<=I_video_vsync;
   O_vdieo_data<=pixel_valid?pixel:24'd0;
   pixel_valid<=I_video_rd_en;pixel<=0;
   if(!ready_sync[1])seen_not_ready<=1;
   if(frame_start)begin decided<=0;use_frame<=0;poison<=0;seen_not_ready<=0;phase<=0;end
   else if(I_video_rd_en)begin
     if(!decided)begin decided<=1;use_frame<=ready_sync[1]&&seen_not_ready&&!fifo_reset;end
     if(can_display&&(!boundary||qcount!=0))begin pixel<=assembled;phase<=phase+1'b1;end
     else if(can_display)poison<=1;
   end
   if(fifo_reset)begin
     qcount<=0;pending<=0;phase<=0;current<=0;
     if(decided&&use_frame)poison<=1;
   end else begin
     pending<=pop;
     case({pending,take})
       2'b10:begin if(qcount==0)q0<=fifo_data;else q1<=fifo_data;qcount<=qcount+1'b1;end
       2'b01:begin q0<=q1;qcount<=qcount-1'b1;end
       2'b11:begin if(qcount==1)q0<=fifo_data;else begin q0<=q1;q1<=fifo_data;end end
     endcase
     if(take)current<=q0;
   end
 end
endmodule
