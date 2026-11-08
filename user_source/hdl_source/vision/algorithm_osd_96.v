`timescale 1ns/1ps
// Four pixels share ONE small 5x7 glyph-row lookup (16x16 character cells).
// Overlay AFTER detection and centered cropping, BEFORE packing/DDR. Stored
// pixels carry their own labels through dropped/repeated frames without CDC.
// No added latency, RAM, DSP, handshake or frame/line marker changes.
module algorithm_osd_96 #(
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720,
    parameter PANEL_X=16,PANEL_Y=IMG_HEIGHT-48, SPLIT_LABELS=0, HEADER_MARGIN=16,UI_STYLE=0,HEADER_Y=16,ROLE_AWARE=0
)(
    input I_clk,I_rst_n,I_valid,I_start,
    input [95:0] I_data,input [3:0] I_mode,input [1:0] I_leader,
    output [95:0] O_data
);
    reg [10:0] x,y;
    reg [3:0] frame_mode;
    reg [1:0] frame_leader;
    wire [10:0] px=I_start?11'd0:x,py=I_start?11'd0:y;
    wire [3:0] mode=I_start?I_mode:frame_mode;
    wire [1:0] leader=I_start?I_leader:frame_leader;
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n)begin x<=0;y<=0;frame_mode<=0;frame_leader<=0;end
        else begin
            if(I_start)begin x<=0;y<=0;frame_mode<=I_mode;frame_leader<=I_leader;end
            if(I_valid)begin
                if(px==IMG_WIDTH-4)begin x<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;end
                else begin x<=px+4;y<=py;end
            end
        end
    // PANEL_X must be a multiple of 16, width a multiple of four.
    wire [10:0] lx=px-PANEL_X,ly=py-PANEL_Y;
    wire panel=I_valid && px>=PANEL_X && px<PANEL_X+512 &&
        py>=PANEL_Y && py<PANEL_Y+32;
    wire [1:0] cell_index=lx[8:7];
    wire [2:0] column=lx[6:4];
    wire raw_mode=mode==0 && cell_index==0;
    wire enabled=raw_mode || mode[cell_index];
    wire background=1'b0; // Enabled effects are visible on the color half.
    wire both_detectors=mode[2] && mode[1];
    wire roi=cell_index==2 && both_detectors && (ROLE_AWARE==0 || leader==1);
    wire assist=ROLE_AWARE!=0 && cell_index==1 && both_detectors && leader==2;
    wire tracking=ROLE_AWARE!=0 && both_detectors &&
        ((cell_index==1 && leader==1)||(cell_index==2 && leader==2));
    reg [63:0] text;
    always @* begin
        if(!ly[4])begin
            case(cell_index)
                0:text=raw_mode?"RAW     ":"CARTOON ";
                1:text="TARGET  ";
                2:text="MOTION  ";
                3:text="EDGE    ";
            endcase
        end else if(!enabled)text="OFF     ";
        else if(background)text="BG      ";
        else if(roi)text="ROI     ";
        else if(assist)text="ASSIST  ";
        else if(tracking)text="TRACK   ";
        else if(cell_index==3)text="L/R     ";
        else text="ON      ";
    end
    wire [10:0] header_x=px-(px>=IMG_WIDTH/2?IMG_WIDTH/2+HEADER_MARGIN:HEADER_MARGIN);
    wire header=SPLIT_LABELS!=0 && mode[3] && I_valid &&
        py>=HEADER_Y && py<HEADER_Y+16 && header_x<128;
    wire [63:0] header_text=px>=IMG_WIDTH/2?"BINARY  ":"COLOR   ";
    wire [7:0] character=header?header_text[63-header_x[6:4]*8-:8]:text[63-column*8-:8];
    wire [3:0] glyph_y=header?py[3:0]:ly[3:0];
    wire [4:0] row_bits;
    algorithm_osd_font u_font(.I_char(character),.I_row(glyph_y[3:1]),.O_bits(row_bits));
    wire [23:0] ink=UI_STYLE!=0?(!enabled?24'h64748b:!ly[4]?24'hdce8f2:(roi||assist)?24'h61afef:24'h56d6b5):
        !enabled?24'h787878:background?24'hffbc40:
        !ly[4]?24'hffffff:(roi||assist)?24'h40dfff:24'h60ff80;
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1)begin:g_pixel
        wire [3:0] sub_x=header?({header_x[3:2],2'b00}+lane):({lx[3:2],2'b00}+lane);
        wire lit=sub_x[3:1]<5 && row_bits[4-sub_x[3:1]] && glyph_y<14;
        wire [11:0] pixel_x={1'b0,px}+lane;
        wire divider=SPLIT_LABELS!=0 && mode[3] && I_valid &&
            pixel_x>=IMG_WIDTH/2-2 && pixel_x<IMG_WIDTH/2+2;
        wire accent=UI_STYLE!=0 && panel && (ly==31 || lx[6:2]==31);
        wire [23:0] backdrop=UI_STYLE!=0?24'h101b29:24'h101820;
        assign O_data[(3-lane)*24+:24]=divider?(UI_STYLE!=0?24'h3b8995:24'h00ffff):
            header?(lit?(UI_STYLE!=0?24'hdce8f2:24'hffffff):backdrop):
            panel?(accent?(enabled?24'h3b8995:24'h29374a):lit?ink:backdrop):I_data[(3-lane)*24+:24];
    end endgenerate
endmodule

// Only the letters used above. A single asynchronous LUT table, no ERAM.
module algorithm_osd_font(input [7:0] I_char,input [2:0] I_row,output reg [4:0] O_bits);
    reg [34:0] glyph;
    always @* begin
        case(I_char)
            "A":glyph=35'b01110_10001_10001_11111_10001_10001_10001;
            "B":glyph=35'b11110_10001_10001_11110_10001_10001_11110;
            "C":glyph=35'b01110_10001_10000_10000_10000_10001_01110;
            "D":glyph=35'b11110_10001_10001_10001_10001_10001_11110;
            "E":glyph=35'b11111_10000_10000_11110_10000_10000_11111;
            "F":glyph=35'b11111_10000_10000_11110_10000_10000_10000;
            "G":glyph=35'b01110_10001_10000_10111_10001_10001_01110;
            "I":glyph=35'b01110_00100_00100_00100_00100_00100_01110;
            "K":glyph=35'b10001_10010_10100_11000_10100_10010_10001;
            "L":glyph=35'b10000_10000_10000_10000_10000_10000_11111;
            "M":glyph=35'b10001_11011_10101_10101_10001_10001_10001;
            "N":glyph=35'b10001_11001_10101_10011_10001_10001_10001;
            "O":glyph=35'b01110_10001_10001_10001_10001_10001_01110;
            "R":glyph=35'b11110_10001_10001_11110_10100_10010_10001;
            "S":glyph=35'b01111_10000_10000_01110_00001_00001_11110;
            "T":glyph=35'b11111_00100_00100_00100_00100_00100_00100;
            "W":glyph=35'b10001_10001_10001_10101_10101_11011_10001;
            "Y":glyph=35'b10001_10001_01010_00100_00100_00100_00100;
            "/":glyph=35'b00001_00010_00010_00100_01000_01000_10000;
            default:glyph=0;
        endcase
        case(I_row)
            0:O_bits=glyph[34:30];1:O_bits=glyph[29:25];
            2:O_bits=glyph[24:20];3:O_bits=glyph[19:15];
            4:O_bits=glyph[14:10];5:O_bits=glyph[9:5];
            6:O_bits=glyph[4:0];default:O_bits=0;
        endcase
    end
endmodule
