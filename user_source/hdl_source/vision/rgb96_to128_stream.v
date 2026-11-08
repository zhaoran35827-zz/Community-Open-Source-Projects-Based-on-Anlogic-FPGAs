`timescale 1ns/1ps
// Four accepted 96-bit beats become three 128-bit beats. Idle clocks never
// discard residual bits. Frame start is an early control pulse, as before.
module rgb96_to128_stream(
 input I_clk,I_rst_n,I_start,I_valid,input [95:0] I_data,
 output reg O_start,O_valid,output reg [127:0] O_data
);
 reg [95:0] previous;
 reg [1:0] phase;
 wire [1:0] at=I_start?2'd0:phase;
 always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin phase<=0;previous<=0;O_start<=0;O_valid<=0;O_data<=0;end
   else begin
     O_start<=I_start;O_valid<=0;
     if(I_start)phase<=0;
     if(I_valid)begin
       previous<=I_data;phase<=at+1'b1;
       case(at)
         1:begin O_valid<=1;O_data<={previous,I_data[95:64]};end
         2:begin O_valid<=1;O_data<={previous[63:0],I_data[95:32]};end
         3:begin O_valid<=1;O_data<={previous[31:0],I_data};end
         default:begin end
       endcase
     end
   end
 end
endmodule
