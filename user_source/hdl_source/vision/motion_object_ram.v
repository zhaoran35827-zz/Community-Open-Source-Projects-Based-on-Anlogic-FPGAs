`timescale 1ns/1ps
// Explicit one-clock ERAM. No reset/clear loop over the memory contents.
// A is the serial worker port, B is the non-stalling capture port.
module motion_object_ram #(parameter WIDTH=18,WORDS=14400,AB=$clog2(WORDS),TWO_PORTS=1)(
 input I_clk,input A_enable,A_write,input [AB-1:0] A_address,
 input [WIDTH-1:0] A_data,output [WIDTH-1:0] A_q,
 input B_enable,B_write,input [AB-1:0] B_address,
 input [WIDTH-1:0] B_data,output [WIDTH-1:0] B_q);
 localparam LB=WIDTH>20?9:10,DEPTH=1<<LB;
 localparam BANKS=(WORDS+DEPTH-1)/DEPTH,BB=BANKS<2?1:$clog2(BANKS);
 reg [BB-1:0] a_bank,b_bank;
 wire [BANKS*WIDTH-1:0] aq,bq;
 always @(posedge I_clk)begin
  if(A_enable)a_bank<=A_address>>LB;
  if(B_enable)b_bank<=B_address>>LB;
 end
 assign A_q=aq[a_bank*WIDTH+:WIDTH];
 assign B_q=bq[b_bank*WIDTH+:WIDTH];
 wire [LB-1:0] aa=A_address,ba=B_address;
 genvar b;
 generate for(b=0;b<BANKS;b=b+1)begin:g_bank
  PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),
   .ADDR_WIDTH_A(LB),.ADDR_WIDTH_B(LB),.DATA_DEPTH_A(DEPTH),.DATA_DEPTH_B(DEPTH),
   .MODE(TWO_PORTS!=0?"DP":"SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
   .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
   .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
   .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
   .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
   .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory(
   .clka(I_clk),.cea(A_enable&&(A_address>>LB)==b),.wea(A_write),.addra(aa),.dia(A_data),.doa(aq[b*WIDTH+:WIDTH]),
   .ocea(1'b0),.rsta(1'b0),.bea(1'b0),
   .clkb(I_clk),.ceb(B_enable&&(B_address>>LB)==b),.web(B_write),.addrb(ba),.dib(B_data),.dob(bq[b*WIDTH+:WIDTH]),
   .oceb(1'b0),.rstb(1'b0),.beb(1'b0),
   .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 end endgenerate
endmodule
