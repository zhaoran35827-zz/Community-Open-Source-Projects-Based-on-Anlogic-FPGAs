`timescale 1ns/1ps
// Centered 3x3 majority. Delay RGB by one row so mask and image agree.
// Replicate image edges. Flush the final row during vertical blanking.
// Four consecutive pixels/beat; horizontal blanking allowed, no backpressure.
module mask_cleanup_96 #(parameter IMG_WIDTH=1280,IMG_HEIGHT=720,RGB_BLOCK_RAM=0)(
 input I_clk,I_rst_n,I_valid,I_frame_start,I_last,
 input [95:0] I_data,input [3:0] I_mask,
 output O_valid,O_frame_start,O_last,
 output [95:0] O_data,output [3:0] O_mask,output [3:0] O_raw_mask);
 localparam BEATS=IMG_WIDTH/4;
 reg [3:0] mask_line[0:BEATS-1],mask_old[0:BEATS-1];
 reg [15:0] beat,row,flush_beat;
 reg flushing,sof_d;
 wire sof=I_frame_start && !sof_d;
 wire [15:0] at=sof?16'd0:beat;
 wire [15:0] iy=sof?16'd0:row;
 wire source=I_valid || flushing;
 wire [15:0] address=flushing?flush_beat:at;
 wire [95:0] read_rgb;
 // Only the RGB delay storage changes implementation; all mask windows,
 // markers, majority decisions and output pipeline registers are unchanged.
 generate if(RGB_BLOCK_RAM!=0)begin:g_rgb_block
   localparam ADDR_BITS=$clog2(BEATS);
   PH1P_LOGIC_ERAM #(
     .DATA_WIDTH_A(96),.DATA_WIDTH_B(96),
     .ADDR_WIDTH_A(ADDR_BITS),.ADDR_WIDTH_B(ADDR_BITS),
     .DATA_DEPTH_A(2**ADDR_BITS),.DATA_DEPTH_B(2**ADDR_BITS),
     .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
     .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("NORMAL"),
     .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
     .OREGSET_A("RESET"),.OREGSET_B("RESET"),
     .SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
     .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
     .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")
   ) rgb_ram (
     .doa(read_rgb),.dia(I_data),.addra(address[ADDR_BITS-1:0]),
     .cea(source),.ocea(1'b0),.clka(I_clk),.wea(I_valid && !flushing),.rsta(1'b0),.bea(1'b0),
     .dob(),.dib(96'd0),.addrb({ADDR_BITS{1'b0}}),
     .ceb(1'b0),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
     .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0)
   );
 end else begin:g_rgb_inferred
   reg [95:0] rgb_line[0:BEATS-1];
   reg [95:0] read_rgb_reg;
   assign read_rgb=read_rgb_reg;
   always @(posedge I_clk)begin
     if(source)read_rgb_reg<=rgb_line[address];
     if(I_valid && !flushing)rgb_line[at]<=I_data;
   end
 end endgenerate
 reg [3:0] read_top,read_mid,read_bot;
 reg read_valid,read_first,read_last,read_sof;
 // Synchronous read/write memories; no array reset (row validity handles it).
 always @(posedge I_clk) begin
   if(source) begin
     read_mid<=mask_line[address];
     read_top<=(!flushing && iy==1)?mask_line[address]:mask_old[address];
     read_bot<=flushing?mask_line[address]:I_mask;
   end
   if(I_valid && !flushing) begin
     mask_line[at]<=I_mask;
     mask_old[at]<=mask_line[at];
   end
 end
 always @(posedge I_clk or negedge I_rst_n) begin
   if(!I_rst_n) begin
     beat<=0;row<=0;flush_beat<=0;flushing<=0;sof_d<=0;
     read_valid<=0;read_first<=0;read_last<=0;read_sof<=0;
   end else begin
     sof_d<=I_frame_start;
     read_valid<=source && (flushing || iy!=0);
     read_first<=address==0;read_last<=address==BEATS-1;
     read_sof<=!flushing && iy==1 && address==0 && I_valid;
     if(sof) begin beat<=0;row<=0;flushing<=0;end
     if(I_valid && !flushing) begin
       if(at==BEATS-1) begin
         beat<=0;row<=iy+1'b1;
         if(iy==IMG_HEIGHT-1) begin flushing<=1;flush_beat<=0;end
       end else begin beat<=at+1'b1;row<=iy;end
     end
     if(flushing) begin
       if(flush_beat==BEATS-1) begin flushing<=0;flush_beat<=0;end
       else flush_beat<=flush_beat+1'b1;
     end
   end
 end
 // One-beat horizontal lookahead, exactly as in the original cleanup stage.
 reg [95:0] rgb_d;
 reg [3:0] top_d,mid_d,bot_d;
 reg valid_d,first_d,last_d,sof_out;
 reg left_top,left_mid,left_bot;
 always @(posedge I_clk or negedge I_rst_n) begin
   if(!I_rst_n) begin
     rgb_d<=0;top_d<=0;mid_d<=0;bot_d<=0;
     valid_d<=0;first_d<=0;last_d<=0;sof_out<=0;
     left_top<=0;left_mid<=0;left_bot<=0;
   end else begin
     rgb_d<=read_rgb;top_d<=read_top;mid_d<=read_mid;bot_d<=read_bot;
     valid_d<=read_valid;first_d<=read_first;last_d<=read_last;sof_out<=read_sof;
     left_top<=top_d[0];left_mid<=mid_d[0];left_bot<=bot_d[0];
   end
 end
 assign O_valid=valid_d;assign O_frame_start=sof_out;assign O_last=last_d && valid_d;assign O_data=rgb_d;
 assign O_raw_mask=valid_d?mid_d:4'd0;
 genvar p;
 generate for(p=0;p<4;p=p+1)begin:g_pixel
   wire [2:0] a,b,c;
   if(p==0)begin
     assign a={first_d?top_d[3]:left_top,top_d[3],top_d[2]};
     assign b={first_d?mid_d[3]:left_mid,mid_d[3],mid_d[2]};
     assign c={first_d?bot_d[3]:left_bot,bot_d[3],bot_d[2]};
   end else if(p==3)begin
     assign a={top_d[1],top_d[0],last_d?top_d[0]:read_top[3]};
     assign b={mid_d[1],mid_d[0],last_d?mid_d[0]:read_mid[3]};
     assign c={bot_d[1],bot_d[0],last_d?bot_d[0]:read_bot[3]};
   end else begin
     assign a=top_d[4-p -:3];assign b=mid_d[4-p -:3];assign c=bot_d[4-p -:3];
   end
   wire [3:0] sum={3'b0,a[0]}+{3'b0,a[1]}+{3'b0,a[2]}+
                  {3'b0,b[0]}+{3'b0,b[1]}+{3'b0,b[2]}+
                  {3'b0,c[0]}+{3'b0,c[1]}+{3'b0,c[2]};
   assign O_mask[3-p]=valid_d && sum>=5;
 end endgenerate
endmodule
