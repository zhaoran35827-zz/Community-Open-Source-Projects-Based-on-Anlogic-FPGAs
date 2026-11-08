`timescale 1ns/1ps
// HX1P35A manual p18-21 and official sm_scan: DIG and SEL are active LOW.
// O_display[7:0]={dp,g,f,e,d,c,b,a}, [15:8]=SEL[7:0] (SEL0 leftmost).
// SW6 must select the numeric display. BUZZER is active LOW; SW5 enables it.
module traffic_board_io #(
    parameter integer CLK_HZ=50000000,
    parameter integer SLOT_HZ=2000
)(
    input I_clk,I_rst_n,I_enable,
    input [2:0] I_phase,
    input [4:0] I_seconds,
    input I_fault,I_block_alarm,
    output wire [15:0] O_display,
    output wire O_buzzer_n
);
    localparam integer SLOT_DIV=CLK_HZ/SLOT_HZ;
    localparam integer CW=(SLOT_DIV<2)?1:$clog2(SLOT_DIV);
    localparam integer SETTLE_CYCLES=(CLK_HZ/1000000<1)?1:CLK_HZ/1000000;
    reg [CW-1:0] slot_count;
    reg [2:0] digit;
    reg [9:0] beep_slots;
    reg [7:0] segments_n,select_n;
    reg frame_enable,frame_fault;
    reg [2:0] frame_phase;
    reg [4:0] frame_seconds;
    reg [3:0] character;
    reg [3:0] tens,ones;
    wire slot_end=slot_count==SLOT_DIV-1;
    wire show_a=frame_phase==2 || frame_phase==3;
    wire show_b=frame_phase==4 || frame_phase==5;
    // The display snapshot changes only with all digits off at an 8-slot wrap.
    // Direct enable gating also blanks promptly on traffic OFF/reset.
    assign O_display=(I_rst_n && I_enable && frame_enable)?{select_n,segments_n}:16'hffff;
    // Active buzzer: ~256 ms ON / ~256 ms OFF, using the existing scan enable.
    assign O_buzzer_n=!(I_rst_n && I_enable && I_block_alarm && !beep_slots[9]);

    function [7:0] glyph;
        input [3:0] ch;
        begin
            case(ch)
                0:glyph=8'hc0;1:glyph=8'hf9;2:glyph=8'ha4;3:glyph=8'hb0;
                4:glyph=8'h99;5:glyph=8'h92;6:glyph=8'h82;7:glyph=8'hf8;
                8:glyph=8'h80;9:glyph=8'h90;
                10:glyph=8'h88; // A
                11:glyph=8'h83; // b
                12:glyph=8'hbf; // -
                13:glyph=8'h86; // E
                14:glyph=8'haf; // r
                default:glyph=8'hff; // blank, including decimal point
            endcase
        end
    endfunction
    always @* begin
        if(frame_seconds>=30)begin tens=3;ones=frame_seconds-30;end
        else if(frame_seconds>=20)begin tens=2;ones=frame_seconds-20;end
        else if(frame_seconds>=10)begin tens=1;ones=frame_seconds-10;end
        else begin tens=0;ones=frame_seconds;end
        case(digit)
            0:character=10;
            4:character=11;
            1,5:character=frame_fault?13:15;
            default:begin
                if(frame_fault)character=14;
                else if((digit<4 && show_a) || (digit>=4 && show_b))
                    character=digit[0]?ones:tens;
                else character=12;
            end
        endcase
    end
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            slot_count<=0;digit<=0;beep_slots<=0;
            segments_n<=8'hff;select_n<=8'hff;
            frame_enable<=0;frame_fault<=0;frame_phase<=0;frame_seconds<=0;
        end else begin
            if(!I_enable || !I_block_alarm)beep_slots<=0;
            else if(slot_end)beep_slots<=beep_slots+1'b1;
            if(slot_end)begin
                slot_count<=0;select_n<=8'hff;digit<=digit+1'b1;
                if(digit==7)begin
                    frame_enable<=I_enable;frame_fault<=I_fault || I_block_alarm;
                    frame_phase<=I_phase;frame_seconds<=I_seconds;
                end
            end else begin
                slot_count<=slot_count+1'b1;
                if(slot_count==0)begin select_n<=8'hff;segments_n<=glyph(character);end
                if(slot_count==SETTLE_CYCLES)select_n<=~(8'b1<<digit);
            end
        end
    end
endmodule
