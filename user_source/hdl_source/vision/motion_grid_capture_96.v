`timescale 1ns/1ps
// Side branch: true 4x4 RGB averages. Pixels and video markers are never delayed
// or stalled by this module. O_end accompanies the final averaged grid cell.
module motion_grid_capture_96 #(parameter IMG_WIDTH=640,IMG_HEIGHT=360,
 COLS=IMG_WIDTH/4,ROWS=IMG_HEIGHT/4,XB=$clog2(COLS),YB=$clog2(ROWS))(
 input I_clk,I_rst_n,I_valid,I_start,input [95:0] I_data,
 output reg O_valid,O_end,output reg [14:0] O_rgb,
 output reg [XB-1:0] O_x,output reg [YB-1:0] O_y);
 localparam RB=XB<1?1:XB;
 reg [XB-1:0] x;reg [$clog2(IMG_HEIGHT)-1:0] y;
 wire [XB-1:0] xx=I_start?0:x;
 wire [$clog2(IMG_HEIGHT)-1:0] yy=I_start?0:y;
 wire [9:0] sr={2'b0,I_data[23:16]}+{2'b0,I_data[47:40]}+{2'b0,I_data[71:64]}+{2'b0,I_data[95:88]};
 wire [9:0] sg={2'b0,I_data[15:8]}+{2'b0,I_data[39:32]}+{2'b0,I_data[63:56]}+{2'b0,I_data[87:80]};
 wire [9:0] sb={2'b0,I_data[7:0]}+{2'b0,I_data[31:24]}+{2'b0,I_data[55:48]}+{2'b0,I_data[79:72]};
 reg p_valid;reg [1:0] p_subrow;reg [XB-1:0] p_x;reg [YB-1:0] p_y;
 reg [29:0] p_sums;
 wire [35:0] row_q;
 wire [11:0] rr={2'b0,p_sums[29:20]}+(p_subrow==0?12'd0:row_q[35:24]);
 wire [11:0] gg={2'b0,p_sums[19:10]}+(p_subrow==0?12'd0:row_q[23:12]);
 wire [11:0] bb={2'b0,p_sums[9:0]}+(p_subrow==0?12'd0:row_q[11:0]);
 // A 36-bit x 256 accumulator maps to one native 20K block (not FFs).
 line_history_ram #(.WIDTH(36),.ADDR_BITS(RB)) rows(
  .clka(I_clk),.wea(p_valid&&p_subrow!=3),.addra(p_x),.dia({rr,gg,bb}),
  .clkb(I_clk),.addrb(xx),.dob(row_q));
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin x<=0;y<=0;p_valid<=0;p_subrow<=0;p_x<=0;p_y<=0;p_sums<=0;
  O_valid<=0;O_end<=0;O_rgb<=0;O_x<=0;O_y<=0;end
 else begin
  p_valid<=I_valid;O_valid<=p_valid&&p_subrow==3;O_end<=p_valid&&p_subrow==3&&p_x==COLS-1&&p_y==ROWS-1;
  if(I_start)begin x<=0;y<=0;end
  if(I_valid)begin
   p_sums<={sr,sg,sb};p_x<=xx;p_y<=yy>>2;p_subrow<=yy[1:0];
   if(xx==COLS-1)begin x<=0;y<=yy==IMG_HEIGHT-1?0:yy+1'b1;end
   else begin x<=xx+1'b1;y<=yy;end
  end
  if(p_valid&&p_subrow==3)begin
   O_rgb<={rr[11:7],gg[11:7],bb[11:7]};O_x<=p_x;O_y<=p_y;
  end
 end
endmodule
