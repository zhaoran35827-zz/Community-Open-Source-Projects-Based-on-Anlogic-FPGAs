`timescale 1ns/1ps
// Two frame maps with independent capture and worker ports. Capture never
// waits for a worker. Only a same-address visited-write/compare-read collision
// stalls the worker writer; immutable RGB is never overwritten by the worker.
module motion_grid_pair_96 #(parameter WIDTH=640,HEIGHT=360,COLS=WIDTH/4,ROWS=HEIGHT/4,
 AB=$clog2(COLS*ROWS),XB=$clog2(COLS),YB=$clog2(ROWS))(
 input I_clk,I_rst_n,I_clear,I_valid,I_start,input [95:0] I_rgb,input [7:0] I_frame,
 output reg O_complete,O_bank,output reg [7:0] O_frame,
 input W_bank,W_enable,W_write,input [AB-1:0] W_address,input [17:0] W_data,
 output [17:0] W_q,output W_write_ready);
 wire gv,ge;wire [14:0] grgb;wire [XB-1:0] gx;wire [YB-1:0] gy;
 motion_grid_capture_96 #(.IMG_WIDTH(WIDTH),.IMG_HEIGHT(HEIGHT)) average(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(I_valid),.I_start(I_start),.I_data(I_rgb),
  .O_valid(gv),.O_end(ge),.O_rgb(grgb),.O_x(gx),.O_y(gy));
 reg capture_bank,has_previous;reg [7:0] capture_frame;
 reg pending,pending_end,pending_bank,pending_warm;reg [14:0] pending_rgb;
 reg [AB-1:0] pending_address;reg [7:0] pending_frame;
 wire [AB-1:0] compare_address=gy*COLS+gx;
 wire [35:0] capture_q,worker_q;
 wire [17:0] previous=pending_bank?capture_q[17:0]:capture_q[35:18];
 function [1:0] seed;input [14:0] now,old;integer r,g,b,lum;
 begin r={1'b0,now[14:10]}-{1'b0,old[14:10]};g={1'b0,now[9:5]}-{1'b0,old[9:5]};b={1'b0,now[4:0]}-{1'b0,old[4:0]};
  lum=r+2*g+b;
  seed=lum>=8?1:lum<=-8?2:((r<0?-r:r)+(g<0?-g:g)+(b<0?-b:b)>=6?3:0);
 end endfunction
 wire [17:0] captured_word={1'b0,(pending_warm?seed(pending_rgb,previous[14:0]):2'b00),pending_rgb};
 assign W_q=W_bank?worker_q[35:18]:worker_q[17:0];
 assign W_write_ready=!(gv&&W_bank!=capture_bank&&W_address==compare_address);
 genvar f;
 generate for(f=0;f<2;f=f+1)begin:g_frame
  wire wr=pending&&pending_bank==f;
  wire rd=gv&&capture_bank!=f;
  motion_object_ram #(.WIDTH(18),.WORDS(COLS*ROWS),.AB(AB)) memory(
   .I_clk(I_clk),.A_enable(W_enable&&W_bank==f&&(!W_write||W_write_ready)),.A_write(W_write),.A_address(W_address),.A_data(W_data),.A_q(worker_q[f*18+:18]),
   .B_enable(wr||rd),.B_write(wr),.B_address(wr?pending_address:compare_address),.B_data(captured_word),.B_q(capture_q[f*18+:18]));
 end endgenerate
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin capture_bank<=0;has_previous<=0;capture_frame<=0;pending<=0;pending_end<=0;pending_bank<=0;pending_warm<=0;pending_rgb<=0;pending_address<=0;pending_frame<=0;O_complete<=0;O_bank<=0;O_frame<=0;end
 else begin
  O_complete<=pending&&pending_end;
  pending<=gv;
  if(I_clear)has_previous<=0;
  if(I_start)begin capture_bank<=!capture_bank;capture_frame<=I_frame;end
  if(gv)begin pending_end<=ge;pending_bank<=capture_bank;pending_warm<=has_previous&&!I_clear;pending_rgb<=grgb;pending_address<=compare_address;pending_frame<=capture_frame;end
  if(pending&&pending_end)begin has_previous<=!I_clear;O_bank<=pending_bank;O_frame<=pending_frame;end
 end
endmodule
