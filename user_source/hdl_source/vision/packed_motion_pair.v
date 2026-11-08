`timescale 1ns/1ps
// Two immutable-per-frame RGB332 histories. Five samples per 40-bit word.
// COLS must be a multiple of five: row translations become constant word adds.
module packed_motion_pair #(parameter COLS=320,ROWS=180)(
 input I_clk,I_rst_n,I_start,I_event,input [7:0] I_signature,
 input [9:0] I_col,I_row,input signed [2:0] I_dx,I_dy,
 output [7:0] O_previous,output reg O_in_bounds);
 localparam WORDS=COLS*ROWS/5,BANKS=(WORDS+1023)/1024;
 localparam AW=$clog2(WORDS),BW=(BANKS<2?1:$clog2(BANKS)),RW=BW+1;
 reg image_bank;reg [AW-1:0] word_address;reg [2:0] slot;
 reg [31:0] collected;reg [RW-1:0] read_bank;reg [2:0] read_slot;
 wire wb=I_start?!image_bank:image_bank;
 wire [AW-1:0] at=I_start?0:word_address;
 wire [2:0] pos=I_start?0:slot;
 wire signed [11:0] rx=$signed({2'b0,I_col})+I_dx;
 wire signed [11:0] ry=$signed({2'b0,I_row})+I_dy;
 wire in_bounds=rx>=0 && rx<COLS && ry>=0 && ry<ROWS;
 wire signed [4:0] shifted_slot=$signed({2'b0,pos})+I_dx;
 wire signed [2:0] carry_word=shifted_slot<0?-3'sd1:(shifted_slot>4?3'sd1:3'sd0);
 wire [2:0] rslot=shifted_slot<0?shifted_slot+5:(shifted_slot>4?shifted_slot-5:shifted_slot);
 wire signed [AW+3:0] rat=$signed({1'b0,at})+$signed(I_dy)*(COLS/5)+carry_word;
 wire [BW-1:0] wr_bank=at>>10,rd_bank=rat>>10;
 // Assignment extends small test geometries rather than reading above AW.
 wire [9:0] write_local=at,read_local=rat;
 wire [2*BANKS*40-1:0] read_words;
 wire [39:0] previous_word=read_words[read_bank*40+:40];
 assign O_previous=O_in_bounds?previous_word[read_slot*8+:8]:8'd0;
 reg [39:0] write_word;
 always @* begin write_word={8'd0,collected};write_word[pos*8+:8]=I_signature;end
 genvar f,b;
 generate for(f=0;f<2;f=f+1)begin:g_image
 for(b=0;b<BANKS;b=b+1)begin:g_bank
 localparam USED_WORDS=(b==BANKS-1)?WORDS-b*1024:1024;
 localparam BANK_AW=(USED_WORDS<=512)?9:10;
 localparam BANK_DEPTH=1<<BANK_AW;
 wire wr=I_event && wb==f && wr_bank==b && pos==4;
 wire rd=I_event && in_bounds && wb!=f && rd_bank==b;
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(40),.DATA_WIDTH_B(40),
 .ADDR_WIDTH_A(BANK_AW),.ADDR_WIDTH_B(BANK_AW),.DATA_DEPTH_A(BANK_DEPTH),.DATA_DEPTH_B(BANK_DEPTH),
 .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
 .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),.OREGSET_A("RESET"),.OREGSET_B("RESET"),
 .SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
 .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory(
 .doa(read_words[(f*BANKS+b)*40+:40]),.dia(write_word),.addra(wr?write_local[BANK_AW-1:0]:read_local[BANK_AW-1:0]),
 .cea(wr || rd),.ocea(1'b0),.clka(I_clk),.wea(wr),.rsta(1'b0),.bea(1'b0),
 .dob(),.dib(40'd0),.addrb({BANK_AW{1'b0}}),.ceb(1'b0),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 end end endgenerate
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin image_bank<=0;word_address<=0;slot<=0;collected<=0;read_bank<=0;read_slot<=0;O_in_bounds<=0;end
 else begin
 if(I_start)begin image_bank<=!image_bank;word_address<=0;slot<=0;end
 if(I_event)begin
 read_bank<=(wb?0:BANKS)+rd_bank;read_slot<=rslot;O_in_bounds<=in_bounds;
 if(pos<4)collected[pos*8+:8]<=I_signature;
 if(pos==4)begin word_address<=at+1'b1;slot<=0;end
 else begin word_address<=at;slot<=pos+1'b1;end
 end end
endmodule
