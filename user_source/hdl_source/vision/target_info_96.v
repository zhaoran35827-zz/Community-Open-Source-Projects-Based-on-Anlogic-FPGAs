`timescale 1ns/1ps
// In-stream annotations share the detector clock and previous-frame record.
// Rendering before DDR avoids unsynchronized multi-bit result crossings.
module target_info_96 #(
    parameter HUD_X = 375,
    parameter HUD_Y = 160,ADAPT_LAYOUT=0
) (
    input wire I_valid,
    input wire [95:0] I_data,
    input wire [15:0] I_pixel_x, I_pixel_y,
    input wire I_box_valid,
    input wire [15:0] I_xmin, I_xmax, I_ymin, I_ymax,
    output wire [95:0] O_data,input I_split
);
    wire [16:0] sum_x = {1'b0,I_xmin} + {1'b0,I_xmax};
    wire [16:0] sum_y = {1'b0,I_ymin} + {1'b0,I_ymax};
    wire [15:0] center_x = sum_x[16:1];
    wire [15:0] center_y = sum_y[16:1];
    // Fixed 1280x720 input; 12-bit coordinates fit four decimal digits.
    function [15:0] to_bcd;
        input [11:0] value;
        integer bit_index, digit;
        reg [15:0] result;
        begin
            result=0;
            for(bit_index=11;bit_index>=0;bit_index=bit_index-1) begin
                for(digit=0;digit<4;digit=digit+1)
                    if(result[digit*4 +:4]>=5)
                        result[digit*4 +:4]=result[digit*4 +:4]+4'd3;
                result={result[14:0],value[bit_index]};
            end
            to_bcd=result;
        end
    endfunction
    wire [15:0] bcd_x=to_bcd(center_x[11:0]);
    wire [15:0] bcd_y=to_bcd(center_y[11:0]);
    function [7:0] status_char;
        input [3:0] column;
        input found;
        begin
            status_char=8'h20;
            if(found) begin
                case(column)
                    0:status_char="T"; 1:status_char="A"; 2:status_char="R";
                    3:status_char="G"; 4:status_char="E"; 5:status_char="T";
                endcase
            end else begin
                case(column)
                    0:status_char="N"; 1:status_char="O";
                    3:status_char="T"; 4:status_char="A"; 5:status_char="R";
                    6:status_char="G"; 7:status_char="E"; 8:status_char="T";
                endcase
            end
        end
    endfunction
    function [7:0] coord_char;
        input [3:0] column;
        input [15:0] bx,by;
        input found;
        begin
            coord_char=8'h20;
            if(found) begin
                case(column)
                    0:coord_char="X"; 1:coord_char=":";
                    2:coord_char=8'h30+bx[15:12]; 3:coord_char=8'h30+bx[11:8];
                    4:coord_char=8'h30+bx[7:4]; 5:coord_char=8'h30+bx[3:0];
                    7:coord_char="Y"; 8:coord_char=":";
                    9:coord_char=8'h30+by[15:12]; 10:coord_char=8'h30+by[11:8];
                    11:coord_char=8'h30+by[7:4]; 12:coord_char=8'h30+by[3:0];
                endcase
            end
        end
    endfunction
    wire [15:0] hud_x=ADAPT_LAYOUT!=0?(I_split?16'd704:16'd608):HUD_X;
    wire [15:0] hud_y=ADAPT_LAYOUT!=0?(I_split?16'd208:16'd120):HUD_Y;
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1) begin:g_lane
        wire [15:0] px=I_pixel_x+lane;
        wire [15:0] lx=px-hud_x;
        wire [15:0] ly=I_pixel_y-hud_y;
        wire panel=px>=hud_x && px<hud_x+208 && I_pixel_y>=hud_y && I_pixel_y<hud_y+40;
        wire first_line=ly<16;
        wire second_line=ly>=24 && ly<40;
        wire [3:0] char_col=lx[7:4];
        wire [7:0] character=first_line?status_char(char_col,I_box_valid):
            coord_char(char_col,bcd_x,bcd_y,I_box_valid);
        wire [3:0] font_row=first_line?ly[3:0]:(ly-16'd24);
        wire [15:0] bits;
        osd_char_lib font(.I_char(character),.I_row(font_row),.O_row_bits(bits));
        wire text_on=panel && (first_line || second_line) && bits[15-lx[3:0]];
        wire [15:0] dx=px>=center_x?px-center_x:center_x-px;
        wire [15:0] dy=I_pixel_y>=center_y?I_pixel_y-center_y:center_y-I_pixel_y;
        wire cross_on=I_box_valid && ((dx<=8 && dy<=1)||(dy<=8 && dx<=1));
        assign O_data[(3-lane)*24 +:24]=!I_valid?I_data[(3-lane)*24 +:24]:
            panel?(text_on?24'hfff200:24'h202020):
            cross_on?24'hff00ff:I_data[(3-lane)*24 +:24];
    end endgenerate
endmodule
