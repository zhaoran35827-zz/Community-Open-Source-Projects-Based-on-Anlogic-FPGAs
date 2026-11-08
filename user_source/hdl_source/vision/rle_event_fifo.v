`timescale 1ns/1ps
// Three events per video beat, one write port per striped RAM bank.
// 48 entries preserve/increase the previous 32-event capacity. No RAM reset.
module rle_event_fifo #(parameter WIDTH=41,FAST_BANKS=0)(
    input I_clk,I_rst_n,I_clear,I_pop,
    input [1:0] I_count,
    input [WIDTH-1:0] I_data0,I_data1,I_data2,
    output [WIDTH-1:0] O_head,
    output reg [6:0] O_count
);
    reg [1:0] wb,rb;
    reg [3:0] wa,ra;
    wire [1:0] start_bank=I_clear?2'd0:wb;
    wire [3:0] start_addr=I_clear?4'd0:wa;
    wire [2:0] advance={1'b0,start_bank}+{1'b0,I_count};
    wire [WIDTH-1:0] read_bank[0:2];
    assign O_head=(rb==0)?read_bank[0]:(rb==1)?read_bank[1]:read_bank[2];
    genvar b;
    generate for(b=0;b<3;b=b+1)begin:g_bank
        reg write_en;
        reg [3:0] write_addr;
        reg [WIDTH-1:0] write_data;
        if(FAST_BANKS!=0)begin:g_direct
        // Bank b wraps solely according to wb>b. Count controls write
        // enable, never the RAM address or data mux. Preserve wb==3 as well.
        localparam PREV=(b+2)%3, PREV2=(b+1)%3;
        always @*begin
            write_addr=I_clear?4'd0:(wa+(wb>b));
            write_en=I_rst_n && ((I_clear || wb==3)?(I_count>b):
                ((wb==b && I_count>=1) || (wb==PREV && I_count>=2) ||
                 (wb==PREV2 && I_count==3)));
            if(I_clear || wb==3)begin
                case(b)
                    0:write_data=I_data0;
                    1:write_data=I_data1;
                    2:write_data=I_data2;
                endcase
            end else write_data=wb==b?I_data0:wb==PREV?I_data1:I_data2;
        end
        end else begin:g_legacy
        integer n,pos;
        always @*begin
            write_en=0;write_addr=start_addr;write_data=0;
            for(n=0;n<3;n=n+1)begin
                pos=start_bank+n;
                if(n<I_count && (pos==b || pos==b+3))begin
                    write_en=I_rst_n;
                    write_addr=start_addr+(pos>=3);
                    case(n)
                        0:write_data=I_data0;
                        1:write_data=I_data1;
                        2:write_data=I_data2;
                    endcase
                end
            end
        end
        end
        PH1P_LOGIC_DRAM #(.DATA_WIDTH_W(WIDTH),.DATA_WIDTH_R(WIDTH),
            .ADDR_WIDTH_W(4),.ADDR_WIDTH_R(4),.DATA_DEPTH_W(16),.DATA_DEPTH_R(16),
            .READREG("DISABLE")) memory (
            .di(write_data),.waddr(write_addr),.we(write_en),.wclk(I_clk),
            .rclk(I_clk),.rrst(1'b0),.rce(1'b0),.raddr(ra),.rdoq(),.rdo(read_bank[b]));
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin wb<=0;rb<=0;wa<=0;ra<=0;O_count<=0;end
        else begin
            if(I_clear)begin rb<=0;ra<=0;O_count<=I_count;end
            else begin
                O_count<=O_count+I_count-(I_pop?1'b1:1'b0);
                if(I_pop)begin
                    if(rb==2)begin rb<=0;ra<=ra+1'b1;end
                    else rb<=rb+1'b1;
                end
            end
            if(I_clear || I_count!=0)begin
                if(advance>=3)begin wb<=advance-3;wa<=start_addr+1'b1;end
                else begin wb<=advance;wa<=start_addr;end
            end
        end
    end
endmodule
