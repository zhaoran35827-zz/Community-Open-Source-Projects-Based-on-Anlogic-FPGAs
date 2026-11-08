`timescale 1ns/1ps
// Exact signed affine interpolation. Four products/divides share ONE DSP and
// one restoring divider instead of four parallel32-bit combinational chains.
module motion_vector_at #(parameter WIDTH=640,HEIGHT=360,LIVE_VECTORS=0)(
 input I_clk,I_rst_n,I_begin,I_abort,input [47:0] I_x,I_y,
 input signed [11:0] I_x0,I_x1,I_y0,I_y1,
 output reg O_done,output reg signed [11:0] O_x,O_y);
 localparam IDLE=0,PREP=1,PRODUCT=2,DIV_START=3,DIVIDE=4,ACCUM=5;
 reg [2:0] state;reg component,axis;
 reg [47:0] vectors_x,vectors_y;
 reg signed [12:0] position_x,position_y;
 reg signed [13:0] gradient_x,gradient_y;
 reg signed [15:0] accumulator;
 // Native caller holds the completed camera snapshot throughout the query.
 wire [47:0] vectors=LIVE_VECTORS!=0?(component?I_y:I_x):(component?vectors_y:vectors_x);
 wire signed [11:0] a=vectors[11:0],b=vectors[23:12],c=vectors[35:24],d=vectors[47:36];
 wire signed [13:0] gradient=axis?gradient_y:gradient_x;
 wire signed [12:0] position=axis?position_y:position_x;
 wire signed [26:0] product=gradient*position;
 reg signed [26:0] numerator;
 reg [26:0] dividend,quotient;reg [12:0] remainder,denominator;
 reg negative;reg [4:0] bit_count;
 wire [13:0] trial={remainder,dividend[26]};
 wire subtract=trial>={1'b0,denominator};
 wire signed [26:0] signed_quotient=negative?-$signed(quotient):$signed(quotient);
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin state<=IDLE;component<=0;axis<=0;vectors_x<=0;vectors_y<=0;position_x<=0;position_y<=0;gradient_x<=0;gradient_y<=0;accumulator<=0;
  numerator<=0;dividend<=0;quotient<=0;remainder<=0;denominator<=0;negative<=0;bit_count<=0;O_done<=0;O_x<=0;O_y<=0;end
 else begin
  O_done<=0;
  if(I_abort)state<=IDLE;
  else case(state)
   IDLE:if(I_begin)begin
    if(LIVE_VECTORS==0)begin vectors_x<=I_x;vectors_y<=I_y;end
    position_x<=$signed(I_x0)+$signed(I_x1)-WIDTH;position_y<=$signed(I_y0)+$signed(I_y1)-HEIGHT;
    component<=0;state<=PREP;
   end
   PREP:begin
    gradient_x<=$signed(b)+$signed(d)-$signed(a)-$signed(c);gradient_y<=$signed(c)+$signed(d)-$signed(a)-$signed(b);
    accumulator<=($signed(a)+$signed(b)+$signed(c)+$signed(d))/4;axis<=0;state<=PRODUCT;
   end
   PRODUCT:begin numerator<=product;state<=DIV_START;end
   DIV_START:begin
    negative<=numerator<0;dividend<=numerator<0?-numerator:numerator;quotient<=0;remainder<=0;denominator<=axis?2*HEIGHT:2*WIDTH;bit_count<=0;state<=DIVIDE;
   end
   DIVIDE:begin
    remainder<=subtract?trial-denominator:trial;quotient<={quotient[25:0],subtract};dividend<={dividend[25:0],1'b0};
    if(bit_count==26)state<=ACCUM;else bit_count<=bit_count+1'b1;
   end
   ACCUM:begin
    accumulator<=accumulator+signed_quotient;
    if(!axis)begin axis<=1;state<=PRODUCT;end
    else if(!component)begin O_x<=accumulator+signed_quotient;component<=1;state<=PREP;end
    else begin O_y<=accumulator+signed_quotient;O_done<=1;state<=IDLE;end
   end
  endcase
 end
endmodule
