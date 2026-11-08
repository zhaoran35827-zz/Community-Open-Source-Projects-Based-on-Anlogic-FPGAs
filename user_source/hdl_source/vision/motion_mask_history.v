`timescale 1ns/1ps
// Packed persistent tile mask: 20 samples/word. Two ERAMs at 1280x720/8x4.
// Read on every frame; update only on comparison frames. This preserves exact
// tile-level green rendering and ROI coordinates between decimated comparisons.
module motion_mask_history #(parameter SAMPLES=28800)(
    input I_clk,I_rst_n,I_start,I_event,I_update,I_bit,
    output O_previous
);
    localparam WORDS=(SAMPLES+19)/20,BANKS=(WORDS+1023)/1024;
    localparam AW=(WORDS<2)?1:$clog2(WORDS),BW=(BANKS<2)?1:$clog2(BANKS);
    reg [AW-1:0] address,read_address;
    reg [4:0] slot,read_slot;
    reg [BW-1:0] read_bank;
    reg [19:0] collected;
    wire [AW-1:0] at=I_start?{AW{1'b0}}:address;
    wire [4:0] pos=I_start?5'd0:slot;
    wire [BANKS*20-1:0] read_words;
    wire [19:0] previous_word=read_words[read_bank*20+:20];
    assign O_previous=previous_word[read_slot];
    wire last_slot=read_slot==19 || (read_address==WORDS-1 && read_slot==(SAMPLES-1)%20);
    reg [19:0] write_word;
    always @* begin write_word=collected;write_word[read_slot]=I_bit;end
    genvar b;
    generate for(b=0;b<BANKS;b=b+1)begin:g_bank
        PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),
            .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
            .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
            .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
            .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
            .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
            .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
            .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
            .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory (
            .doa(read_words[b*20+:20]),.dia(20'd0),.addra(at%1024),
            .cea(I_event && (at>>10)==b),.ocea(1'b0),.clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),
            .dob(),.dib(write_word),.addrb(read_address%1024),
            .ceb(I_update && last_slot && (read_address>>10)==b),.oceb(1'b0),.clkb(I_clk),
            .web(I_update && last_slot && (read_address>>10)==b),.rstb(1'b0),.beb(1'b0),
            .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin address<=0;slot<=0;read_address<=0;read_slot<=0;read_bank<=0;collected<=0;end
        else begin
            if(I_start)begin address<=0;slot<=0;end
            if(I_event)begin
                read_address<=at;read_slot<=pos;read_bank<=at>>10;
                if(pos==19 || (at==WORDS-1 && pos==(SAMPLES-1)%20))begin address<=at+1'b1;slot<=0;end
                else begin address<=at;slot<=pos+1'b1;end
            end
            if(I_update)collected[read_slot]<=I_bit;
        end
    end
endmodule
