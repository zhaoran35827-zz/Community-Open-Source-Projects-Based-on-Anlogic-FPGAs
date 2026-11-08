`timescale 1ns/1ps
// Serialize five consecutive RGB332 samples into one logical 40-bit RAM word.
// Samples are read before replacement; one read per accepted sample, one write
// per five samples. Six logical 40x1024 banks map to twelve 20x1024 ERAMs.
// This module preserves the original synchronous one-cycle read latency.
module packed_motion_history #(parameter SAMPLES=28800)(
    input I_clk,I_rst_n,I_start,I_event,
    input [7:0] I_signature,output [7:0] O_previous
);
    localparam WORDS=(SAMPLES+4)/5,BANKS=(WORDS+1023)/1024;
    localparam AW=(WORDS<2)?1:$clog2(WORDS);
    localparam BW=(BANKS<2)?1:$clog2(BANKS);
    reg [AW-1:0] word_address;
    reg [2:0] slot,read_slot;
    reg [BW-1:0] read_bank;
    reg [31:0] collected;
    wire [AW-1:0] at=I_start?{AW{1'b0}}:word_address;
    wire [2:0] pos=I_start?3'd0:slot;
    wire [BW-1:0] bank=at>>10;
    wire [9:0] offset=at%1024;
    reg [39:0] write_word;
    always @* begin
        write_word={8'd0,collected};
        write_word[pos*8 +:8]=I_signature;
    end
    wire word_end=pos==4 || (at==WORDS-1 && pos==(SAMPLES-1)%5);
    wire [BANKS*40-1:0] read_words;
    wire [39:0] previous_word=read_words[read_bank*40 +:40];
    assign O_previous=previous_word[read_slot*8 +:8];
    genvar b;
    generate for(b=0;b<BANKS;b=b+1)begin:g_bank
        PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(40),.DATA_WIDTH_B(40),
            .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
            .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
            .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("NORMAL"),
            .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
            .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
            .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
            .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
            .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory (
            .doa(read_words[b*40 +:40]),.dia(write_word),.addra(offset),
            .cea(I_event && bank==b),.ocea(1'b0),.clka(I_clk),
            .wea(I_event && bank==b && word_end),.rsta(1'b0),.bea(1'b0),
            .dob(),.dib(40'd0),.addrb(10'd0),.ceb(1'b0),.oceb(1'b0),
            .clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
            .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            word_address<=0;slot<=0;collected<=0;read_slot<=0;read_bank<=0;
        end else begin
            if(I_start)begin word_address<=0;slot<=0;end
            if(I_event)begin
                read_slot<=pos;read_bank<=bank;
                if(pos<4)collected[pos*8 +:8]<=I_signature;
                if(word_end)begin word_address<=at+1'b1;slot<=0;end
                else begin word_address<=at;slot<=pos+1'b1;end
            end
        end
    end
endmodule
