`timescale 1ns/1ps
// Shared serial seeded region reconstruction. Queue/mapped colours live in
// ERAM, not a frame-sized register array. Colour growth fills unchanged
// interiors; camera-relative movement is validated later, before any merge.
module motion_region_worker #(parameter COLS=160,ROWS=90,AB=$clog2(COLS*ROWS),XB=$clog2(COLS),YB=$clog2(ROWS))(
 input I_clk,I_rst_n,I_begin,I_abort,
 output reg O_busy,O_done,O_overflow,
 output reg M_enable,M_write,output reg [AB-1:0] M_address,output reg [17:0] M_data,
 input [17:0] M_q,input I_write_ready,
 output reg O_proposal,input I_proposal_ready,
 output reg [XB:0] O_x0,O_x1,output reg [YB:0] O_y0,O_y1,
 output reg [14:0] O_area,O_colour);
 localparam IDLE=0,SCAN=1,SCAN_Q=2,ROOT_WRITE=3,POP=4,POP_Q=5,NODE=6,NODE_Q=7,
  NEIGHBOUR=8,NEIGHBOUR_Q=9,VISIT=10,EMIT=11,NEXT=12,
  PRE_SCALE=13,PRE_SAT=14,GUARD=15,CHANNEL_A=16,CHANNEL_B=17,CHANNEL_C=18,REJECT=19;
 reg [4:0] state;reg [1:0] pass;
 reg [XB-1:0] scan_x,node_x,next_x,root_x;
 reg [YB-1:0] scan_y,node_y,next_y,root_y;
 reg [14:0] root_rgb,here_rgb;
 reg [1:0] polarity,direction;
 reg [17:0] saved;
 reg [9:0] head,tail;reg [10:0] queued;
 reg [14:0] area,hits;reg [XB-1:0] xmin,xmax;reg [YB-1:0] ymin,ymax;
 reg region_bad;
 reg [4:0] root_max,root_spread,here_max,other_max,other_spread;
 reg [1:0] channel,root_axis;
 reg [9:0] root_scale,here_scale,other_saturation,root_saturation,product_a,product_b;
 reg [4:0] mul_a0,mul_b0,mul_a1,mul_b1;
 wire [9:0] product0=mul_a0*mul_b0,product1=mul_a1*mul_b1;
 wire [3:0] shift={channel,2'b00}+channel;
 wire [4:0] other_channel=saved[14:0]>>shift,here_channel=here_rgb>>shift,root_channel=root_rgb>>shift;
 wire [3:0] root_shift={root_axis,2'b00}+root_axis;
 wire [4:0] other_dominant=saved[14:0]>>root_shift;
 wire [6:0] root_spread3=({2'b0,root_spread}<<1)+{2'b0,root_spread};
 wire grey_root=root_spread3<(root_max==0?5'd1:root_max);
 wire [9:0] cross_delta=product_a>product_b?product_a-product_b:product_b-product_a;
 wire [11:0] here_scale3=({2'b0,here_scale}<<1)+{2'b0,here_scale};
 reg [XB-1:0] address_x;reg [YB-1:0] address_y;
 wire [XB+YB-1:0] queue_q;
 wire q_write=(state==ROOT_WRITE||(state==VISIT&&queued<1024))&&I_write_ready;
 wire [XB+YB-1:0] queue_data=state==ROOT_WRITE?{root_y,root_x}:{next_y,next_x};
 motion_object_ram #(.WIDTH(XB+YB),.WORDS(1024),.AB(10)) queue(
  .I_clk(I_clk),.A_enable(q_write),.A_write(1'b1),.A_address(tail),.A_data(queue_data),.A_q(),
  .B_enable(state==POP),.B_write(1'b0),.B_address(head),.B_data({(XB+YB){1'b0}}),.B_q(queue_q));
 function [4:0] hi;input [14:0] c;reg [4:0] v;begin v=c[14:10]>c[9:5]?c[14:10]:c[9:5];hi=v>c[4:0]?v:c[4:0];end endfunction
 function [4:0] lo;input [14:0] c;reg [4:0] v;begin v=c[14:10]<c[9:5]?c[14:10]:c[9:5];lo=v<c[4:0]?v:c[4:0];end endfunction
 function integer mag;input integer v;begin mag=v<0?-v:v;end endfunction
 wire [4:0] seed_max=hi(M_q[14:0]);
 wire [5:0] seed_spread={1'b0,seed_max}-{1'b0,lo(M_q[14:0])};
 wire [7:0] spread3=({2'b0,seed_spread}<<1)+{2'b0,seed_spread};
 wire pass_ok=pass==2||(pass==0?{seed_spread,1'b0}>seed_max:spread3>seed_max);
 reg neighbour_ok;
 always @* begin
  mul_a0=0;mul_b0=0;mul_a1=0;mul_b1=0;
  case(state)
   PRE_SCALE:begin mul_a0=root_max;mul_b0=other_max;mul_a1=here_max;mul_b1=other_max;end
   PRE_SAT:begin mul_a0=other_spread;mul_b0=root_max;mul_a1=root_spread;mul_b1=other_max;end
   CHANNEL_A:begin mul_a0=other_channel;mul_b0=here_max;mul_a1=here_channel;mul_b1=other_max;end
   CHANNEL_B:begin mul_a0=other_channel;mul_b0=root_max;mul_a1=root_channel;mul_b1=other_max;end
  endcase
  next_x=node_x;next_y=node_y;neighbour_ok=1;
  case(direction)
   0:begin next_x=node_x-1'b1;neighbour_ok=node_x!=0;end
   1:begin next_x=node_x+1'b1;neighbour_ok=node_x<COLS-1;end
   2:begin next_y=node_y-1'b1;neighbour_ok=node_y!=0;end
   3:begin next_y=node_y+1'b1;neighbour_ok=node_y<ROWS-1;end
  endcase
  M_enable=0;M_write=0;M_data={1'b1,saved[16:0]};address_x=scan_x;address_y=scan_y;
  case(state)
   SCAN:M_enable=1;
   ROOT_WRITE:begin M_enable=I_write_ready;M_write=1;address_x=root_x;address_y=root_y;end
   NODE:begin M_enable=1;address_x=node_x;address_y=node_y;end
   NEIGHBOUR:begin M_enable=neighbour_ok;address_x=next_x;address_y=next_y;end
   VISIT:begin M_enable=I_write_ready&&queued<1024;M_write=1;address_x=next_x;address_y=next_y;end
  endcase
  M_address=COLS==160?(({7'd0,address_y}<<7)+({7'd0,address_y}<<5)+address_x):address_y*COLS+address_x;
 end
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  state<=IDLE;O_busy<=0;O_done<=0;O_overflow<=0;O_proposal<=0;
  pass<=0;scan_x<=0;scan_y<=0;node_x<=0;node_y<=0;root_x<=0;root_y<=0;
  root_rgb<=0;here_rgb<=0;polarity<=0;direction<=0;saved<=0;head<=0;tail<=0;queued<=0;
  area<=0;hits<=0;xmin<=0;xmax<=0;ymin<=0;ymax<=0;region_bad<=0;
  O_x0<=0;O_x1<=0;O_y0<=0;O_y1<=0;O_area<=0;O_colour<=0;
  root_max<=0;root_spread<=0;here_max<=0;other_max<=0;other_spread<=0;channel<=0;root_axis<=0;
  root_scale<=0;here_scale<=0;other_saturation<=0;root_saturation<=0;product_a<=0;product_b<=0;
 end else begin
  O_done<=0;
  if(I_abort)begin state<=IDLE;O_busy<=0;O_proposal<=0;end
  else case(state)
   IDLE:if(I_begin)begin pass<=0;scan_x<=0;scan_y<=0;state<=SCAN;O_busy<=1;O_overflow<=0;end
   SCAN:state<=SCAN_Q;
   SCAN_Q:if(!M_q[17]&&M_q[16:15]!=0&&pass_ok)begin
    root_rgb<=M_q[14:0];polarity<=M_q[16:15];root_x<=scan_x;root_y<=scan_y;saved<=M_q;
    root_max<=hi(M_q[14:0]);root_spread<=hi(M_q[14:0])-lo(M_q[14:0]);
    root_axis<=M_q[14:10]>=M_q[9:5]&&M_q[14:10]>=M_q[4:0]?2:M_q[9:5]>=M_q[4:0]?1:0;
    head<=0;tail<=0;queued<=0;area<=0;hits<=0;xmin<=scan_x;xmax<=scan_x;ymin<=scan_y;ymax<=scan_y;region_bad<=0;state<=ROOT_WRITE;
   end else state<=NEXT;
   ROOT_WRITE:if(I_write_ready)begin tail<=tail+1'b1;queued<=1;state<=POP;end
   POP:state<=POP_Q;
   POP_Q:begin node_x<=queue_q[XB-1:0];node_y<=queue_q[XB+:YB];head<=head+1'b1;queued<=queued-1'b1;state<=NODE;end
   NODE:state<=NODE_Q;
   NODE_Q:begin
    here_rgb<=M_q[14:0];here_max<=hi(M_q[14:0]);area<=area+1'b1;if(M_q[16:15]!=0)hits<=hits+1'b1;
    if(node_x<xmin)xmin<=node_x;if(node_x>xmax)xmax<=node_x;
    if(node_y<ymin)ymin<=node_y;if(node_y>ymax)ymax<=node_y;
    direction<=0;state<=NEIGHBOUR;
   end
   NEIGHBOUR:state<=NEIGHBOUR_Q;
   NEIGHBOUR_Q:begin
    if(neighbour_ok&&!M_q[17])begin
     saved<=M_q;other_max<=hi(M_q[14:0]);other_spread<=hi(M_q[14:0])-lo(M_q[14:0]);state<=PRE_SCALE;
    end else if(direction==3)state<=EMIT;else begin direction<=direction+1'b1;state<=NEIGHBOUR;end
   end
   PRE_SCALE:begin root_scale<=product0;here_scale<=product1;state<=PRE_SAT;end
   PRE_SAT:begin other_saturation<=product0;root_saturation<=product1;state<=GUARD;end
   GUARD:if(grey_root)begin
    if({2'b0,other_saturation}>{2'b0,root_saturation}+({2'b0,root_scale}>>3)+4||
       (polarity==1&&{1'b0,other_max}+1<{1'b0,root_max})||
       (polarity==2&&{1'b0,other_max}>{1'b0,root_max}+1)||
       (polarity==3&&mag({1'b0,other_max}-{1'b0,here_max})>3))state<=REJECT;
    else state<=VISIT;
   end else if(root_max<3||here_max<3||other_max<3||{1'b0,other_dominant}+1<{1'b0,other_max}||
     {other_saturation,1'b0}<{1'b0,root_saturation})state<=REJECT;
   else begin channel<=2;state<=CHANNEL_A;end
   CHANNEL_A:begin product_a<=product0;product_b<=product1;state<=CHANNEL_B;end
   CHANNEL_B:if({2'b0,cross_delta}>(here_scale3>>3)+8)state<=REJECT;
   else begin product_a<=product0;product_b<=product1;state<=CHANNEL_C;end
   CHANNEL_C:if({2'b0,cross_delta}>({2'b0,root_scale}>>1)+8)state<=REJECT;
   else if(channel==0)state<=VISIT;else begin channel<=channel-1'b1;state<=CHANNEL_A;end
   REJECT:if(direction==3)state<=EMIT;else begin direction<=direction+1'b1;state<=NEIGHBOUR;end
   VISIT:if(queued==1024)begin region_bad<=1;O_overflow<=1;state<=REJECT;end
   else if(I_write_ready)begin
    tail<=tail+1'b1;queued<=queued+1'b1;
    if(direction==3)state<=EMIT;else begin direction<=direction+1'b1;state<=NEIGHBOUR;end
   end
   EMIT:if(queued!=0)state<=POP;
   else if(!region_bad&&hits>=2&&area>=4&&xmax>xmin&&ymax>ymin&&
       !(xmin==0&&xmax==COLS-1)&&!(ymin==0&&ymax==ROWS-1)&&
       {area,3'b0}>=(xmax-xmin+1)*(ymax-ymin+1))begin
    O_proposal<=1;O_x0<={1'b0,xmin};O_x1<={1'b0,xmax}+1'b1;
    O_y0<={1'b0,ymin};O_y1<={1'b0,ymax}+1'b1;O_area<=area;O_colour<=root_rgb;
    if(O_proposal&&I_proposal_ready)begin O_proposal<=0;state<=NEXT;end
   end else state<=NEXT;
   NEXT:if(scan_x==COLS-1)begin
    scan_x<=0;
    if(scan_y==ROWS-1)begin scan_y<=0;if(pass==2)begin O_done<=1;O_busy<=0;state<=IDLE;end else begin pass<=pass+1'b1;state<=SCAN;end end
    else begin scan_y<=scan_y+1'b1;state<=SCAN;end
   end else begin scan_x<=scan_x+1'b1;state<=SCAN;end
  endcase
 end
endmodule
