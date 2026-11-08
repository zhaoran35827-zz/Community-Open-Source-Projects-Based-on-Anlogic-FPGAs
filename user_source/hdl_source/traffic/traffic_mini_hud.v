`timescale 1ns/1ps
// Pure combinational overlay. No framebuffer, font RAM, multiplier or
// additional video register. The caller supplies frame-latched status and
// the SAME delayed coordinates/de as its existing RGB output register.
// Status: {enable,mode_ok,fault,good,occupied,phase[2:0],seconds[4:0],
//          count_a[2:0],count_b[2:0],wait_a[7:0],wait_b[7:0],reserved[3:0]}.
// Rows: A<count> RYG <seconds> S<occupied>, B<count> RYG <seconds> <health>.
// Health is MODE, FAIL, WAIT or OK. Only the green/yellow side has a timer;
// the opposing red duration depends on actual channel clearance.
module traffic_mini_hud #(parameter UI_STYLE=0,IMG_WIDTH=1280,IMG_HEIGHT=720) (
    input I_de,
    input [11:0] I_x,I_y,
    input [38:0] I_status,
    output reg O_hit,
    output reg [23:0] O_color
);
    localparam [23:0] BG=(UI_STYLE!=0?24'h101b29:24'h101820),TEXT=24'he0e8f0,DIM=24'h405060;
    localparam [23:0] RED=24'hff5050,YELLOW=24'hffdc40,GREEN=24'h40ff80;
    localparam [23:0] A_EDGE=24'h409cff,B_EDGE=24'hd080ff;
    localparam [23:0] CLEAR_EDGE=24'h40d0c0,BUSY_EDGE=24'hff9830;
    wire mode_ok=I_status[37],fault=I_status[36],good=I_status[35];
    wire occupied=I_status[34];
    wire [2:0] phase=I_status[33:31];
    wire [4:0] seconds=I_status[30:26];
    wire healthy=mode_ok && good && !fault;
    wire row_b=I_y>=12'd80;
    wire green_on=healthy && (row_b ? phase==4 : phase==2);
    wire yellow_on=healthy && (row_b ? phase==5 : phase==3);
    wire red_on=!(green_on || yellow_on);
    wire [2:0] count=row_b?I_status[22:20]:I_status[25:23];
    wire [3:0] tens=seconds>=30?4'd3:seconds>=20?4'd2:seconds>=10?4'd1:4'd0;
    wire [4:0] units=seconds>=30?seconds-30:seconds>=20?seconds-20:seconds>=10?seconds-10:seconds;
    localparam PANEL_X=IMG_WIDTH-480;
    localparam A0=IMG_WIDTH/20,A1=IMG_WIDTH*3/8-1,C0=IMG_WIDTH*3/8,C1=IMG_WIDTH*5/8-1;
    localparam B0=IMG_WIDTH*5/8,B1=IMG_WIDTH*19/20-1,Y0=IMG_HEIGHT*2/9,Y1=IMG_HEIGHT*8/9-1;
    wire [11:0] panel_x=I_x-PANEL_X;
    wire [11:0] glyph_y=I_y-(row_b?12'd88:12'd40);
    wire [4:0] cell_x=panel_x[4:0];
    wire [4:0] glyph_x=cell_x-5'd6;
    wire [3:0] glyph_cell=panel_x[8:5];
    reg [7:0] character;
    reg [23:0] ink;
    reg [34:0] glyph;
    reg [4:0] glyph_row;
    // A single shared 5x7 combinational glyph decoder. Font rows run top to
    // bottom, bit 4 is left. All dimensions are shifts/masks, not division.
    function [34:0] font;
        input [7:0] c;
        begin
            case(c)
                "0":font={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
                "1":font={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "2":font={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
                "3":font={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
                "4":font={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
                "5":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
                "6":font={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
                "7":font={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
                "8":font={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
                "9":font={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
                "A":font={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "B":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
                "D":font={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
                "E":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
                "F":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
                "G":font={5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01111};
                "I":font={5'b01110,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "K":font={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
                "L":font={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
                "M":font={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
                "O":font={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "R":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
                "S":font={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
                "T":font={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
                "W":font={5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b11011,5'b10001};
                "Y":font={5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100};
                "-":font={5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000};
                default:font=35'd0;
            endcase
        end
    endfunction
    always @* begin
        character=" ";ink=TEXT;
        case(glyph_cell)
            0:character=row_b?"B":"A";
            1:character=(mode_ok && good)?("0"+count):"-";
            3:begin character="R";ink=red_on?RED:DIM;end
            4:begin character="Y";ink=yellow_on?YELLOW:DIM;end
            5:begin character="G";ink=green_on?GREEN:DIM;end
            7:character=(green_on || yellow_on)?("0"+tens):"-";
            8:character=(green_on || yellow_on)?("0"+units):"-";
            default:begin end
        endcase
        if(!row_b)begin
            if(glyph_cell==10) character="S";
            if(glyph_cell==11)begin character=good?(occupied?"1":"0"):"-";ink=occupied?BUSY_EDGE:CLEAR_EDGE;end
        end else if(glyph_cell>=9 && glyph_cell<=12)begin
            if(!mode_ok)begin
                ink=YELLOW;
                case(glyph_cell)9:character="M";10:character="O";11:character="D";12:character="E";endcase
            end else if(fault)begin
                ink=RED;
                case(glyph_cell)9:character="F";10:character="A";11:character="I";12:character="L";endcase
            end else if(!good)begin
                ink=YELLOW;
                case(glyph_cell)9:character="W";10:character="A";11:character="I";12:character="T";endcase
            end else begin
                ink=GREEN;
                case(glyph_cell)10:character="O";11:character="K";default:character=" ";endcase
            end
        end
        glyph=font(character);
        case(glyph_y[4:2])
            0:glyph_row=glyph[34:30];1:glyph_row=glyph[29:25];
            2:glyph_row=glyph[24:20];3:glyph_row=glyph[19:15];
            4:glyph_row=glyph[14:10];5:glyph_row=glyph[9:5];
            6:glyph_row=glyph[4:0];default:glyph_row=0;
        endcase
        O_hit=0;O_color=BG;
        if(I_status[38] && I_de)begin
            if(I_x>=PANEL_X && I_x<=PANEL_X+415 && I_y>=32 && I_y<=127)begin
                O_hit=1;
                if(UI_STYLE!=0 && (I_x==PANEL_X || I_x==PANEL_X+415 || I_y==32 || I_y==127)) O_color=24'h3b8995;
                else if(cell_x>=6 && cell_x<26 && glyph_y<28 && glyph_row[4-glyph_x[4:2]]) O_color=ink;
            end else if(mode_ok && I_y>=Y0 && I_y<=Y1)begin
                if(I_x>=A0 && I_x<=A1 && (I_x==A0 || I_x==A1 || I_y==Y0 || I_y==Y1))begin
                    O_hit=1;O_color=A_EDGE;
                end else if(I_x>=C0 && I_x<=C1 && (I_x==C0 || I_x==C1 || I_y==Y0 || I_y==Y1))begin
                    O_hit=1;O_color=occupied?BUSY_EDGE:CLEAR_EDGE;
                end else if(I_x>=B0 && I_x<=B1 && (I_x==B0 || I_x==B1 || I_y==Y0 || I_y==Y1))begin
                    O_hit=1;O_color=B_EDGE;
                end
            end
        end
    end
endmodule
