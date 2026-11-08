module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0,
    parameter integer LATENCY_ENABLE = 1,
    parameter TRAFFIC_ENABLE=0,UI_STYLE=0,COMPACT_HEADER=0,
    // Upstream registered read latency measured from O_video_rd_en.
    // The legacy direct-RGB test/source uses0; video_out uses2 clocks.
    parameter PIPELINE_DIAG=0,READ_DATA_DELAY=0
)(
    input wire [38:0] I_traffic_status,
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,
    input wire [15:0] I_capture_frame_cnt,
    // All statistics inputs below are coherent, in I_clk's clock domain.
    input wire [15:0] I_display_frame_cnt,
    input wire        I_display_frame_valid,
    input wire        I_display_frame_update,

    input wire[3:0]   I_debug_status,
    input wire[15:0] I_pipeline_status,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    reg [15:0] diag_frame;
    always @(posedge I_clk or negedge I_rst_n)
      if(!I_rst_n)diag_frame<=0;else if(I_video_user)diag_frame<=I_pipeline_status;
    wire diag_hit=PIPELINE_DIAG && S_x_2d>=64 && S_x_2d<208 && S_y_2d>=64 && S_y_2d<80;
    wire [7:0] diag_dx=S_x_2d-12'd64;
    reg [7:0] diag_char;reg [3:0] diag_nibble;
    always @* begin
      diag_nibble=0;
      case(diag_dx[7:4])
      5:diag_nibble=diag_frame[15:12];6:diag_nibble=diag_frame[11:8];
      7:diag_nibble=diag_frame[7:4];8:diag_nibble=diag_frame[3:0];endcase
      case(diag_dx[7:4])
      0:diag_char="P";1:diag_char="I";2:diag_char="P";3:diag_char="E";4:diag_char=":";
      default:diag_char=diag_nibble<10?(8'd48+diag_nibble):(8'd65+diag_nibble-10);
      endcase
    end
    wire [15:0] diag_font;
    osd_char_lib diag_glyph(.I_char(diag_char),.I_row(S_y_2d[3:0]),.O_row_bits(diag_font));
    reg        S_osd_hit;
    reg [23:0] S_osd_color;
    reg [11:0] S_logo_x;
    reg [11:0] S_logo_y;
    reg [3:0]  S_char_idx;
    reg [2:0]  S_char_row;
    reg [2:0]  S_char_col;
    reg [4:0]  S_char_bits;
    reg        S_char_on;
    reg        S_logo_blue_on;
    reg        S_logo_red_on;
    reg [11:0] S_logo_tmp_y;
    reg [3:0]  S_logo_red_band;
    reg [11:0] S_logo_red_x_start;
    reg [11:0] S_logo_red_x_end;
    reg [15:0] S_logo_addr;
    wire[24:0] S_logo_pixel;
    reg [31:0] S_frame_cnt;
    reg [11:0] S_dyn_x;
    reg [11:0] S_dyn_y;
    reg [3:0]  S_dyn_char;
    reg [2:0]  S_dyn_col;
    reg [2:0]  S_dyn_row;
    reg [4:0]  S_dyn_bits;
    reg        S_dyn_on;
    reg [3:0]  S_dyn_digit_idx;
    reg [11:0] S_dyn_digit_x;
    reg [6:0]  S_dyn_seg;
    reg [3:0]  S_cnt_d0;
    reg [3:0]  S_cnt_d1;
    reg [3:0]  S_cnt_d2;
    reg [3:0]  S_cnt_d3;
    reg [3:0]  S_cnt_d4;
    reg [3:0]  S_cnt_d5;
    reg [2:0]  S_lbl_row;
    reg [2:0]  S_lbl_col;
    reg [1:0]  S_lbl_idx;
    reg [4:0]  S_lbl_bits;
    reg [6:0]  S_lbl_local_x;
    reg [3:0]  S_dyn_dec_digit;
    reg [4:0]  S_cn_local_x;
    reg [4:0]  S_cn_local_y;
    reg [3:0]  S_cn_col;
    reg [3:0]  S_cn_row;
    reg [15:0] S_cn_row_bits;
    reg        S_cn_on;
    reg [9:0]  S_cn_scale_x;
    reg [9:0]  S_cn_scale_y;
    reg [7:0]  S_pf_char;
    reg [4:0]  S_pf_local_x;
    reg [4:0]  S_pf_local_y;
    reg [3:0]  S_pf_col;
    reg [3:0]  S_pf_row;
    wire [15:0] S_pf_row_bits;
    reg        S_pf_on;

    reg S_video_user_1d;
    wire S_display_frame_start = I_video_user && !S_video_user_1d;
    // Snapshot once per displayed frame; the overlay shares the original
    // coordinate and RGB output register, so video latency is unchanged.
    reg [38:0] S_traffic_status;
    wire S_traffic_hit;
    wire [23:0] S_traffic_color;
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n) S_traffic_status <= 39'd0;
        else if(S_display_frame_start) S_traffic_status <= TRAFFIC_ENABLE!=0?I_traffic_status:39'd0;
    traffic_mini_hud #(.UI_STYLE(UI_STYLE),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_traffic_hud (
        .I_de(S_video_de_2d),.I_x(S_x_2d),.I_y(S_y_2d),
        .I_status(S_traffic_status),.O_hit(S_traffic_hit),.O_color(S_traffic_color));

    reg S_display_metadata_fresh;
    reg [15:0] S_display_reference_cnt;
    reg [15:0] S_latency_frames;
    reg S_latency_valid;
    reg S_latency_saturated;
    // Unsigned modulo-65536 difference, valid only within half the range.
    wire [15:0] S_latency_delta = I_capture_frame_cnt - I_display_frame_cnt;
    // SC500CS uics500reg_720p60: capture tags advance at 60 frames/s.
    // This remains a frame-age estimate, rounded to milliseconds. Bound the
    // multiplier/divider input first, including long pauses and tag wrap.
    localparam [15:0] LATENCY_CAPTURE_FPS = 16'd60;
    wire [5:0] S_latency_scale_frames = (S_latency_delta >= LATENCY_CAPTURE_FPS) ?
                                      6'd60 : S_latency_delta[5:0];
    wire [15:0] S_latency_ms_numerator = S_latency_scale_frames * 16'd1000 +
                                       LATENCY_CAPTURE_FPS / 16'd2;
    // Exact 60-fps rounded frame-age conversion for clamped0..60 input.
    // Keep the existing numerator net for debug; this table does not read it.
    function [15:0] F_latency_ms60;
        input [5:0] frames;
        begin
            case(frames)
                6'd0: F_latency_ms60 = 16'd0;
                6'd1: F_latency_ms60 = 16'd17;
                6'd2: F_latency_ms60 = 16'd33;
                6'd3: F_latency_ms60 = 16'd50;
                6'd4: F_latency_ms60 = 16'd67;
                6'd5: F_latency_ms60 = 16'd83;
                6'd6: F_latency_ms60 = 16'd100;
                6'd7: F_latency_ms60 = 16'd117;
                6'd8: F_latency_ms60 = 16'd133;
                6'd9: F_latency_ms60 = 16'd150;
                6'd10: F_latency_ms60 = 16'd167;
                6'd11: F_latency_ms60 = 16'd183;
                6'd12: F_latency_ms60 = 16'd200;
                6'd13: F_latency_ms60 = 16'd217;
                6'd14: F_latency_ms60 = 16'd233;
                6'd15: F_latency_ms60 = 16'd250;
                6'd16: F_latency_ms60 = 16'd267;
                6'd17: F_latency_ms60 = 16'd283;
                6'd18: F_latency_ms60 = 16'd300;
                6'd19: F_latency_ms60 = 16'd317;
                6'd20: F_latency_ms60 = 16'd333;
                6'd21: F_latency_ms60 = 16'd350;
                6'd22: F_latency_ms60 = 16'd367;
                6'd23: F_latency_ms60 = 16'd383;
                6'd24: F_latency_ms60 = 16'd400;
                6'd25: F_latency_ms60 = 16'd417;
                6'd26: F_latency_ms60 = 16'd433;
                6'd27: F_latency_ms60 = 16'd450;
                6'd28: F_latency_ms60 = 16'd467;
                6'd29: F_latency_ms60 = 16'd483;
                6'd30: F_latency_ms60 = 16'd500;
                6'd31: F_latency_ms60 = 16'd517;
                6'd32: F_latency_ms60 = 16'd533;
                6'd33: F_latency_ms60 = 16'd550;
                6'd34: F_latency_ms60 = 16'd567;
                6'd35: F_latency_ms60 = 16'd583;
                6'd36: F_latency_ms60 = 16'd600;
                6'd37: F_latency_ms60 = 16'd617;
                6'd38: F_latency_ms60 = 16'd633;
                6'd39: F_latency_ms60 = 16'd650;
                6'd40: F_latency_ms60 = 16'd667;
                6'd41: F_latency_ms60 = 16'd683;
                6'd42: F_latency_ms60 = 16'd700;
                6'd43: F_latency_ms60 = 16'd717;
                6'd44: F_latency_ms60 = 16'd733;
                6'd45: F_latency_ms60 = 16'd750;
                6'd46: F_latency_ms60 = 16'd767;
                6'd47: F_latency_ms60 = 16'd783;
                6'd48: F_latency_ms60 = 16'd800;
                6'd49: F_latency_ms60 = 16'd817;
                6'd50: F_latency_ms60 = 16'd833;
                6'd51: F_latency_ms60 = 16'd850;
                6'd52: F_latency_ms60 = 16'd867;
                6'd53: F_latency_ms60 = 16'd883;
                6'd54: F_latency_ms60 = 16'd900;
                6'd55: F_latency_ms60 = 16'd917;
                6'd56: F_latency_ms60 = 16'd933;
                6'd57: F_latency_ms60 = 16'd950;
                6'd58: F_latency_ms60 = 16'd967;
                6'd59: F_latency_ms60 = 16'd983;
                6'd60: F_latency_ms60 = 16'd1000;
                default: F_latency_ms60 = 16'd1000;
            endcase
        end
    endfunction
    wire [15:0] S_latency_ms = F_latency_ms60(S_latency_scale_frames);
    wire [9:0] S_latency_clamped = (S_latency_ms > 16'd999) ?
                                   10'd999 : S_latency_ms[9:0];
    reg [9:0] S_latency_binary;
    reg [11:0] S_latency_bcd_work;
    reg [11:0] S_latency_bcd;
    reg [3:0] S_latency_convert_left;
    wire [11:0] S_latency_bcd_next;
    reg [11:0] S_lat_x, S_lat_y;
    reg [7:0] S_lat_char;
    reg [3:0] S_lat_row, S_lat_col;
    wire [15:0] S_lat_row_bits = S_pf_row_bits;
    // HDMI pixel domain: prefix/CNT and LAT windows never overlap. Reuse the
    // existing character table without adding a stage to the pixel pipeline.
    // LAT overrides the table address only in its separate coordinate window.

    localparam LAT_OSD_X = COMPACT_HEADER!=0?12'd352:12'd375;
    localparam LAT_OSD_Y = COMPACT_HEADER!=0?12'd20:12'd152;
    localparam LAT_OSD_W = 12'd144; // nine native 16x16 glyphs: LAT:xxxMS
    localparam LAT_OSD_H = 12'd16;

    localparam LOGO_X = COMPACT_HEADER!=0?12'd64:12'd128;
    localparam LOGO_Y = COMPACT_HEADER!=0?12'd8:12'd0;
    localparam LOGO_W = COMPACT_HEADER!=0?12'd40:12'd160;
    localparam LOGO_H = COMPACT_HEADER!=0?12'd40:12'd160;

    localparam COLOR_LOGO_BLUE   = 24'h1f315c;
    localparam COLOR_LOGO_RED    = 24'hec1c2d;
    localparam COLOR_TEXT        = 24'h00e6ff;
    localparam DYN_OSD_X         = COMPACT_HEADER!=0?12'd128:12'd375;
    localparam DYN_OSD_Y         = COMPACT_HEADER!=0?12'd16:12'd120;
    localparam DYN_OSD_W         = 12'd200;
    localparam DYN_OSD_H         = 12'd24;
    localparam CN_Y_OFFSET       = 12'd2;
    localparam COLOR_DYN_TEXT    = 24'hfff200;
    localparam CHAR_AN           = 8'h80;
    localparam CHAR_LU           = 8'h81;
    localparam CHAR_C            = 8'h43;
    localparam CHAR_N            = 8'h4e;
    localparam CHAR_T            = 8'h54;
    localparam CHAR_0            = 8'h30;
    localparam CHAR_COLON        = 8'h3a;

    function [4:0] F_logo_text_bits;
        input [3:0] I_char;
        input [2:0] I_row;
        begin
            case(I_char)
                4'd0: begin
                    case(I_row) // A
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10001;
                        3'd3: F_logo_text_bits = 5'b11111;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b10001;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd1: begin
                    case(I_row) // N
                        3'd0: F_logo_text_bits = 5'b10001;
                        3'd1: F_logo_text_bits = 5'b11001;
                        3'd2: F_logo_text_bits = 5'b10101;
                        3'd3: F_logo_text_bits = 5'b10011;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b10001;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd2: begin
                    case(I_row) // L
                        3'd0: F_logo_text_bits = 5'b10000;
                        3'd1: F_logo_text_bits = 5'b10000;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10000;
                        3'd4: F_logo_text_bits = 5'b10000;
                        3'd5: F_logo_text_bits = 5'b10000;
                        3'd6: F_logo_text_bits = 5'b11111;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd3: begin
                    case(I_row) // O
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10001;
                        3'd3: F_logo_text_bits = 5'b10001;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd4: begin
                    case(I_row) // G
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10111;
                        3'd4: F_logo_text_bits = 5'b10001;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd5: begin
                    case(I_row) // I
                        3'd0: F_logo_text_bits = 5'b11111;
                        3'd1: F_logo_text_bits = 5'b00100;
                        3'd2: F_logo_text_bits = 5'b00100;
                        3'd3: F_logo_text_bits = 5'b00100;
                        3'd4: F_logo_text_bits = 5'b00100;
                        3'd5: F_logo_text_bits = 5'b00100;
                        3'd6: F_logo_text_bits = 5'b11111;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                4'd6: begin
                    case(I_row) // C
                        3'd0: F_logo_text_bits = 5'b01110;
                        3'd1: F_logo_text_bits = 5'b10001;
                        3'd2: F_logo_text_bits = 5'b10000;
                        3'd3: F_logo_text_bits = 5'b10000;
                        3'd4: F_logo_text_bits = 5'b10000;
                        3'd5: F_logo_text_bits = 5'b10001;
                        3'd6: F_logo_text_bits = 5'b01110;
                        default: F_logo_text_bits = 5'b00000;
                    endcase
                end
                default: F_logo_text_bits = 5'b00000;
            endcase
        end
    endfunction

    // 7-seg bit mapping: {a,b,c,d,e,f,g}
    function [6:0] F_hex7seg;
        input [3:0] I_hex;
        begin
            case(I_hex)
                4'h0: F_hex7seg = 7'b1111110;
                4'h1: F_hex7seg = 7'b0110000;
                4'h2: F_hex7seg = 7'b1101101;
                4'h3: F_hex7seg = 7'b1111001;
                4'h4: F_hex7seg = 7'b0110011;
                4'h5: F_hex7seg = 7'b1011011;
                4'h6: F_hex7seg = 7'b1011111;
                4'h7: F_hex7seg = 7'b1110000;
                4'h8: F_hex7seg = 7'b1111111;
                4'h9: F_hex7seg = 7'b1111011;
                4'ha: F_hex7seg = 7'b1110111;
                4'hb: F_hex7seg = 7'b0011111;
                4'hc: F_hex7seg = 7'b1001110;
                4'hd: F_hex7seg = 7'b0111101;
                4'he: F_hex7seg = 7'b1001111;
                4'hf: F_hex7seg = 7'b1000111;
                default: F_hex7seg = 7'b0000000;
            endcase
        end
    endfunction

    function [4:0] F_cnt_label_bits;
        input [1:0] I_idx;
        input [2:0] I_row;
        begin
            case(I_idx)
                2'd0: begin // C
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b01110;
                        3'd1: F_cnt_label_bits = 5'b10001;
                        3'd2: F_cnt_label_bits = 5'b10000;
                        3'd3: F_cnt_label_bits = 5'b10000;
                        3'd4: F_cnt_label_bits = 5'b10000;
                        3'd5: F_cnt_label_bits = 5'b10001;
                        3'd6: F_cnt_label_bits = 5'b01110;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                2'd1: begin // N
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b10001;
                        3'd1: F_cnt_label_bits = 5'b11001;
                        3'd2: F_cnt_label_bits = 5'b10101;
                        3'd3: F_cnt_label_bits = 5'b10011;
                        3'd4: F_cnt_label_bits = 5'b10001;
                        3'd5: F_cnt_label_bits = 5'b10001;
                        3'd6: F_cnt_label_bits = 5'b10001;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                2'd2: begin // T
                    case(I_row)
                        3'd0: F_cnt_label_bits = 5'b11111;
                        3'd1: F_cnt_label_bits = 5'b00100;
                        3'd2: F_cnt_label_bits = 5'b00100;
                        3'd3: F_cnt_label_bits = 5'b00100;
                        3'd4: F_cnt_label_bits = 5'b00100;
                        3'd5: F_cnt_label_bits = 5'b00100;
                        3'd6: F_cnt_label_bits = 5'b00100;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
                default: begin // ':'
                    case(I_row)
                        3'd2: F_cnt_label_bits = 5'b00100;
                        3'd5: F_cnt_label_bits = 5'b00100;
                        default: F_cnt_label_bits = 5'b00000;
                    endcase
                end
            endcase
        end
    endfunction

    // Minimal 16x16 Chinese bitmap rows for "an" and "lu".
    function [15:0] F_cn16_row;
        input I_char; // 0: an, 1: lu
        input [3:0] I_row;
        begin
            if(!I_char) begin
                case(I_row)
                    4'd0:  F_cn16_row = 16'h0200;
                    4'd1:  F_cn16_row = 16'h7ff0;
                    4'd2:  F_cn16_row = 16'h4410;
                    4'd3:  F_cn16_row = 16'h0400;
                    4'd4:  F_cn16_row = 16'h7ff0;
                    4'd5:  F_cn16_row = 16'h0840;
                    4'd6:  F_cn16_row = 16'h1040;
                    4'd7:  F_cn16_row = 16'h1c80;
                    4'd8:  F_cn16_row = 16'h0300;
                    4'd9:  F_cn16_row = 16'h0cc0;
                    4'd10: F_cn16_row = 16'h7020;
                    4'd11: F_cn16_row = 16'h0000;
                    4'd12: F_cn16_row = 16'h0000;
                    4'd13: F_cn16_row = 16'h0000;
                    4'd14: F_cn16_row = 16'h0000;
                    4'd15: F_cn16_row = 16'h0000;
                    default: F_cn16_row = 16'h0000;
                endcase
            end
            else begin
                case(I_row)
                    4'd0:  F_cn16_row = 16'h79e0;
                    4'd1:  F_cn16_row = 16'h4a20;
                    4'd2:  F_cn16_row = 16'h4d40;
                    4'd3:  F_cn16_row = 16'h7880;
                    4'd4:  F_cn16_row = 16'h1140;
                    4'd5:  F_cn16_row = 16'h1630;
                    4'd6:  F_cn16_row = 16'h5be0;
                    4'd7:  F_cn16_row = 16'h5220;
                    4'd8:  F_cn16_row = 16'h5220;
                    4'd9:  F_cn16_row = 16'h5be0;
                    4'd10: F_cn16_row = 16'h6220;
                    4'd11: F_cn16_row = 16'h0000;
                    4'd12: F_cn16_row = 16'h0000;
                    4'd13: F_cn16_row = 16'h0000;
                    4'd14: F_cn16_row = 16'h0000;
                    4'd15: F_cn16_row = 16'h0000;
                    default: F_cn16_row = 16'h0000;
                endcase
            end
        end
    endfunction

    function [4:0] F_dyn_char_bits;
        input [3:0] I_char;
        input [2:0] I_row;
        begin
            case(I_char)
                4'h0: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10011; 3'd3:F_dyn_char_bits=5'b10101; 3'd4:F_dyn_char_bits=5'b11001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h1: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00100; 3'd1:F_dyn_char_bits=5'b01100; 3'd2:F_dyn_char_bits=5'b00100; 3'd3:F_dyn_char_bits=5'b00100; 3'd4:F_dyn_char_bits=5'b00100; 3'd5:F_dyn_char_bits=5'b00100; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h2: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b00001; 3'd3:F_dyn_char_bits=5'b00110; 3'd4:F_dyn_char_bits=5'b01000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b11111; default:F_dyn_char_bits=5'b00000; endcase end
                4'h3: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b00001; 3'd2:F_dyn_char_bits=5'b00001; 3'd3:F_dyn_char_bits=5'b01110; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b00001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h4: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00010; 3'd1:F_dyn_char_bits=5'b00110; 3'd2:F_dyn_char_bits=5'b01010; 3'd3:F_dyn_char_bits=5'b10010; 3'd4:F_dyn_char_bits=5'b11111; 3'd5:F_dyn_char_bits=5'b00010; 3'd6:F_dyn_char_bits=5'b00010; default:F_dyn_char_bits=5'b00000; endcase end
                4'h5: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b11110; 3'd3:F_dyn_char_bits=5'b00001; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h6: begin case(I_row) 3'd0:F_dyn_char_bits=5'b00110; 3'd1:F_dyn_char_bits=5'b01000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h7: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b00001; 3'd2:F_dyn_char_bits=5'b00010; 3'd3:F_dyn_char_bits=5'b00100; 3'd4:F_dyn_char_bits=5'b01000; 3'd5:F_dyn_char_bits=5'b01000; 3'd6:F_dyn_char_bits=5'b01000; default:F_dyn_char_bits=5'b00000; endcase end
                4'h8: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b01110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'h9: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b01111; 3'd4:F_dyn_char_bits=5'b00001; 3'd5:F_dyn_char_bits=5'b00010; 3'd6:F_dyn_char_bits=5'b11100; default:F_dyn_char_bits=5'b00000; endcase end
                4'ha: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b11111; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b10001; default:F_dyn_char_bits=5'b00000; endcase end
                4'hb: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'hc: begin case(I_row) 3'd0:F_dyn_char_bits=5'b01110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b10000; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b01110; default:F_dyn_char_bits=5'b00000; endcase end
                4'hd: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11110; 3'd1:F_dyn_char_bits=5'b10001; 3'd2:F_dyn_char_bits=5'b10001; 3'd3:F_dyn_char_bits=5'b10001; 3'd4:F_dyn_char_bits=5'b10001; 3'd5:F_dyn_char_bits=5'b10001; 3'd6:F_dyn_char_bits=5'b11110; default:F_dyn_char_bits=5'b00000; endcase end
                4'he: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b11111; default:F_dyn_char_bits=5'b00000; endcase end
                4'hf: begin case(I_row) 3'd0:F_dyn_char_bits=5'b11111; 3'd1:F_dyn_char_bits=5'b10000; 3'd2:F_dyn_char_bits=5'b10000; 3'd3:F_dyn_char_bits=5'b11110; 3'd4:F_dyn_char_bits=5'b10000; 3'd5:F_dyn_char_bits=5'b10000; 3'd6:F_dyn_char_bits=5'b10000; default:F_dyn_char_bits=5'b00000; endcase end
                default: F_dyn_char_bits = 5'b00000;
            endcase
        end
    endfunction

    // Unified prefix character library (16x16 rows): an/lu/C/N/T/:
    function [15:0] F_osd_char16_row;
        input [7:0] I_char;
        input [3:0] I_row;
        begin
            case(I_char)
                CHAR_AN: begin
                    case(I_row)
                        4'd0:  F_osd_char16_row = 16'h0200;
                        4'd1:  F_osd_char16_row = 16'h7ff0;
                        4'd2:  F_osd_char16_row = 16'h4410;
                        4'd3:  F_osd_char16_row = 16'h0400;
                        4'd4:  F_osd_char16_row = 16'h7ff0;
                        4'd5:  F_osd_char16_row = 16'h0840;
                        4'd6:  F_osd_char16_row = 16'h1040;
                        4'd7:  F_osd_char16_row = 16'h1c80;
                        4'd8:  F_osd_char16_row = 16'h0300;
                        4'd9:  F_osd_char16_row = 16'h0cc0;
                        4'd10: F_osd_char16_row = 16'h7020;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_LU: begin
                    case(I_row)
                        4'd0:  F_osd_char16_row = 16'h79e0;
                        4'd1:  F_osd_char16_row = 16'h4a20;
                        4'd2:  F_osd_char16_row = 16'h4d40;
                        4'd3:  F_osd_char16_row = 16'h7880;
                        4'd4:  F_osd_char16_row = 16'h1140;
                        4'd5:  F_osd_char16_row = 16'h1630;
                        4'd6:  F_osd_char16_row = 16'h5be0;
                        4'd7:  F_osd_char16_row = 16'h5220;
                        4'd8:  F_osd_char16_row = 16'h5220;
                        4'd9:  F_osd_char16_row = 16'h5be0;
                        4'd10: F_osd_char16_row = 16'h6220;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_C: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h03f0;
                        4'd3:  F_osd_char16_row = 16'h0c0c;
                        4'd4:  F_osd_char16_row = 16'h1800;
                        4'd5:  F_osd_char16_row = 16'h3000;
                        4'd6:  F_osd_char16_row = 16'h3000;
                        4'd7:  F_osd_char16_row = 16'h3000;
                        4'd8:  F_osd_char16_row = 16'h3000;
                        4'd9:  F_osd_char16_row = 16'h3000;
                        4'd10: F_osd_char16_row = 16'h1800;
                        4'd11: F_osd_char16_row = 16'h0c0c;
                        4'd12: F_osd_char16_row = 16'h03f0;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_N: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h300c;
                        4'd3:  F_osd_char16_row = 16'h380c;
                        4'd4:  F_osd_char16_row = 16'h3c0c;
                        4'd5:  F_osd_char16_row = 16'h360c;
                        4'd6:  F_osd_char16_row = 16'h330c;
                        4'd7:  F_osd_char16_row = 16'h318c;
                        4'd8:  F_osd_char16_row = 16'h30cc;
                        4'd9:  F_osd_char16_row = 16'h306c;
                        4'd10: F_osd_char16_row = 16'h303c;
                        4'd11: F_osd_char16_row = 16'h301c;
                        4'd12: F_osd_char16_row = 16'h300c;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_T: begin
                    case(I_row)
                        4'd2:  F_osd_char16_row = 16'h3ffc;
                        4'd3:  F_osd_char16_row = 16'h03c0;
                        4'd4:  F_osd_char16_row = 16'h03c0;
                        4'd5:  F_osd_char16_row = 16'h03c0;
                        4'd6:  F_osd_char16_row = 16'h03c0;
                        4'd7:  F_osd_char16_row = 16'h03c0;
                        4'd8:  F_osd_char16_row = 16'h03c0;
                        4'd9:  F_osd_char16_row = 16'h03c0;
                        4'd10: F_osd_char16_row = 16'h03c0;
                        4'd11: F_osd_char16_row = 16'h03c0;
                        4'd12: F_osd_char16_row = 16'h03c0;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                CHAR_COLON: begin
                    case(I_row)
                        4'd6:  F_osd_char16_row = 16'h0180;
                        4'd7:  F_osd_char16_row = 16'h0180;
                        4'd10: F_osd_char16_row = 16'h0180;
                        4'd11: F_osd_char16_row = 16'h0180;
                        default: F_osd_char16_row = 16'h0000;
                    endcase
                end
                default: F_osd_char16_row = 16'h0000;
            endcase
        end
    endfunction

    assign O_video_rd_en = I_video_de;

    generate if(COMPACT_HEADER!=0)begin:g_compact_logo_rom
        // Request one stage ahead of S_x_2d/S_y_2d. The sync ERAM output
        // therefore reaches the unchanged OSD/color/de pipeline on time.
        wire [5:0] logo_x=S_x_1d-LOGO_X,logo_y=S_y_1d-LOGO_Y;
        wire [10:0] address={logo_y,5'b0}+{2'b0,logo_y,3'b0}+{5'b0,logo_x};
        anlogic_logo_compact_sync u_logo(.I_clk(I_clk),.I_addr(address),.O_pixel(S_logo_pixel));
    end else begin:g_original_logo_rom
        anlogic_logo_rom u_anlogic_logo_rom(.I_addr(S_logo_addr),.O_pixel(S_logo_pixel));
    end endgenerate

    osd_char_lib u_osd_char_lib(
        .I_char     ( S_pf_char       ),
        .I_row      ( S_pf_row        ),
        .O_row_bits ( S_pf_row_bits   )
    );

    // HDMI-domain 24-to-16 font scaling: exact for the existing 0..23 range.
    // Small lookup replaces the original wide constant division.
    function [3:0] F_scale24_to16;
        input [4:0] I_position;
        begin
            case(I_position)
                5'd0, 5'd1: F_scale24_to16 = 4'd0;
                5'd2: F_scale24_to16 = 4'd1;
                5'd3, 5'd4: F_scale24_to16 = 4'd2;
                5'd5: F_scale24_to16 = 4'd3;
                5'd6, 5'd7: F_scale24_to16 = 4'd4;
                5'd8: F_scale24_to16 = 4'd5;
                5'd9, 5'd10: F_scale24_to16 = 4'd6;
                5'd11: F_scale24_to16 = 4'd7;
                5'd12, 5'd13: F_scale24_to16 = 4'd8;
                5'd14: F_scale24_to16 = 4'd9;
                5'd15, 5'd16: F_scale24_to16 = 4'd10;
                5'd17: F_scale24_to16 = 4'd11;
                5'd18, 5'd19: F_scale24_to16 = 4'd12;
                5'd20: F_scale24_to16 = 4'd13;
                5'd21, 5'd22: F_scale24_to16 = 4'd14;
                5'd23: F_scale24_to16 = 4'd15;
                default: F_scale24_to16 = 4'd0;
            endcase
        end
    endfunction

    // Ten-cycle binary-to-BCD conversion, outside the pixel rendering path.
    // No /100 or %10 hardware is added to the pixel rendering path.
    function [11:0] F_latency_bcd_step;
        input [11:0] I_bcd;
        input I_bit;
        reg [11:0] S_adjusted;
        begin
            S_adjusted = I_bcd;
            if(I_bcd[3:0] >= 4'd5)
                S_adjusted[3:0] = I_bcd[3:0] + 4'd3;
            if(I_bcd[7:4] >= 4'd5)
                S_adjusted[7:4] = I_bcd[7:4] + 4'd3;
            if(I_bcd[11:8] >= 4'd5)
                S_adjusted[11:8] = I_bcd[11:8] + 4'd3;
            F_latency_bcd_step = {S_adjusted[10:0],I_bit};
        end
    endfunction
    assign S_latency_bcd_next = F_latency_bcd_step(S_latency_bcd_work,
                                                  S_latency_binary[9]);

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_user_1d <= 1'b0;
            S_display_metadata_fresh <= 1'b0;
            S_display_reference_cnt <= 16'd0;
            S_latency_frames <= 16'd0;
            S_latency_valid <= 1'b0;
            S_latency_saturated <= 1'b0;
            S_latency_binary <= 10'd0;
            S_latency_bcd_work <= 12'd0;
            S_latency_bcd <= 12'd0;
            S_latency_convert_left <= 4'd0;
        end
        else begin
            S_video_user_1d <= I_video_user;
            if(I_display_frame_update)
                S_display_metadata_fresh <= 1'b1;
            if(S_display_frame_start) begin
                // Latch once per displayed frame, including repeated images.
                // S_frame_cnt remains the independent display refresh count.
                S_display_metadata_fresh <= 1'b0;
                S_display_reference_cnt <= I_display_frame_cnt;
                S_latency_frames <= S_latency_delta;
                S_latency_valid <= (S_display_metadata_fresh || I_display_frame_update) &&
                                   I_display_frame_valid && !S_latency_delta[15];
                S_latency_saturated <= (S_latency_ms > 16'd999);
                S_latency_binary <= S_latency_clamped;
                S_latency_bcd_work <= 12'd0;
                S_latency_convert_left <= 4'd10;
            end
            else if(S_latency_convert_left != 4'd0) begin
                S_latency_binary <= {S_latency_binary[8:0],1'b0};
                S_latency_bcd_work <= S_latency_bcd_next;
                S_latency_convert_left <= S_latency_convert_left - 4'd1;
                if(S_latency_convert_left == 4'd1)
                    S_latency_bcd <= S_latency_bcd_next;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(S_display_frame_start) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_cnt <= 32'd0;
            S_cnt_d0 <= 4'd0;
            S_cnt_d1 <= 4'd0;
            S_cnt_d2 <= 4'd0;
            S_cnt_d3 <= 4'd0;
            S_cnt_d4 <= 4'd0;
            S_cnt_d5 <= 4'd0;
        end
        else if(S_display_frame_start) begin
            S_frame_cnt <= S_frame_cnt + 32'd1;
            if(S_cnt_d0 == 4'd9) begin
                S_cnt_d0 <= 4'd0;
                if(S_cnt_d1 == 4'd9) begin
                    S_cnt_d1 <= 4'd0;
                    if(S_cnt_d2 == 4'd9) begin
                        S_cnt_d2 <= 4'd0;
                        if(S_cnt_d3 == 4'd9) begin
                            S_cnt_d3 <= 4'd0;
                            if(S_cnt_d4 == 4'd9) begin
                                S_cnt_d4 <= 4'd0;
                                if(S_cnt_d5 == 4'd9)
                                    S_cnt_d5 <= 4'd0;
                                else
                                    S_cnt_d5 <= S_cnt_d5 + 4'd1;
                            end
                            else
                                S_cnt_d4 <= S_cnt_d4 + 4'd1;
                        end
                        else
                            S_cnt_d3 <= S_cnt_d3 + 4'd1;
                    end
                    else
                        S_cnt_d2 <= S_cnt_d2 + 4'd1;
                end
                else
                    S_cnt_d1 <= S_cnt_d1 + 4'd1;
            end
            else
                S_cnt_d0 <= S_cnt_d0 + 4'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end

    always @(*) begin
        S_osd_hit   = 1'b0;
        S_osd_color = 24'd0;
        S_logo_addr = 16'd0;

        S_logo_x = 12'd0;
        S_logo_y = 12'd0;
        S_char_idx = 4'd0;
        S_char_row = 3'd0;
        S_char_col = 3'd0;
        S_char_bits = 5'd0;
        S_char_on = 1'b0;
        S_logo_blue_on = 1'b0;
        S_logo_red_on = 1'b0;
        S_logo_tmp_y = 12'd0;
        S_logo_red_band = 4'd0;
        S_logo_red_x_start = 12'd0;
        S_logo_red_x_end = 12'd0;
        S_dyn_x = 12'd0;
        S_dyn_y = 12'd0;
        S_dyn_char = 4'd0;
        S_dyn_col = 3'd0;
        S_dyn_row = 3'd0;
        S_dyn_bits = 5'd0;
        S_dyn_on = 1'b0;
        S_dyn_digit_idx = 4'd0;
        S_dyn_digit_x = 12'd0;
        S_dyn_seg = 7'd0;
        S_lbl_row = 3'd0;
        S_lbl_col = 3'd0;
        S_lbl_idx = 2'd0;
        S_lbl_bits = 5'd0;
        S_lbl_local_x = 7'd0;
        S_dyn_dec_digit = 4'd0;
        S_cn_local_x = 5'd0;
        S_cn_local_y = 5'd0;
        S_cn_col = 4'd0;
        S_cn_row = 4'd0;
        S_cn_row_bits = 16'd0;
        S_cn_on = 1'b0;
        S_cn_scale_x = 10'd0;
        S_cn_scale_y = 10'd0;
        S_pf_char = 8'd0;
        S_pf_local_x = 5'd0;
        S_pf_local_y = 5'd0;
        S_pf_col = 4'd0;
        S_pf_row = 4'd0;
        S_pf_on = 1'b0;
        S_lat_x = 12'd0;
        S_lat_y = 12'd0;
        S_lat_char = 8'd0;
        S_lat_row = 4'd0;
        S_lat_col = 4'd0;

        if(S_video_de_2d) begin
            if((S_x_2d >= LOGO_X) && (S_x_2d < (LOGO_X + LOGO_W)) &&
               (S_y_2d >= LOGO_Y) && (S_y_2d < (LOGO_Y + LOGO_H))) begin
                S_logo_x = S_x_2d - LOGO_X;
                S_logo_y = S_y_2d - LOGO_Y;
                S_logo_addr = COMPACT_HEADER!=0?{S_logo_y[5:0],2'b00,S_logo_x[5:0],2'b00}:{S_logo_y[7:0],S_logo_x[7:0]};
                if(S_logo_pixel[24]) begin
                    S_osd_hit   = 1'b1;
                    S_osd_color = S_logo_pixel[23:0];
                end
            end

            if((!S_osd_hit) &&
               (S_x_2d >= DYN_OSD_X) && (S_x_2d < (DYN_OSD_X + DYN_OSD_W)) &&
               (S_y_2d >= DYN_OSD_Y) && (S_y_2d < (DYN_OSD_Y + DYN_OSD_H))) begin
                S_dyn_x = S_x_2d - DYN_OSD_X;
                S_dyn_y = S_y_2d - DYN_OSD_Y;

                // Unified prefix render using one character library.
                // an:[0..23], lu:[24..47], C:[52..67], N:[68..83], T:[84..99], :[100..107]
                if((S_x_2d < (DYN_OSD_X + 12'd108)) && (S_y_2d < (DYN_OSD_Y + 12'd24))) begin
                    if((S_x_2d < (DYN_OSD_X + 12'd24)) || ((S_x_2d >= (DYN_OSD_X + 12'd24)) && (S_x_2d < (DYN_OSD_X + 12'd48)))) begin
                        if(S_y_2d >= (DYN_OSD_Y + CN_Y_OFFSET))
                            S_pf_local_y = S_y_2d - (DYN_OSD_Y + CN_Y_OFFSET);
                        else
                            S_pf_local_y = 5'd31;

                        if(S_x_2d < (DYN_OSD_X + 12'd24)) begin
                            S_pf_char = CHAR_AN;
                            S_pf_local_x = S_x_2d - DYN_OSD_X;
                        end
                        else begin
                            S_pf_char = CHAR_LU;
                            S_pf_local_x = S_x_2d - (DYN_OSD_X + 12'd24);
                        end

                        if((S_pf_local_x < 5'd24) && (S_pf_local_y < 5'd24)) begin
                            S_pf_col = F_scale24_to16(S_pf_local_x);
                            S_pf_row = F_scale24_to16(S_pf_local_y);
                            case(S_pf_col)
                                4'd0:  S_pf_on = S_pf_row_bits[15];
                                4'd1:  S_pf_on = S_pf_row_bits[14];
                                4'd2:  S_pf_on = S_pf_row_bits[13];
                                4'd3:  S_pf_on = S_pf_row_bits[12];
                                4'd4:  S_pf_on = S_pf_row_bits[11];
                                4'd5:  S_pf_on = S_pf_row_bits[10];
                                4'd6:  S_pf_on = S_pf_row_bits[9];
                                4'd7:  S_pf_on = S_pf_row_bits[8];
                                4'd8:  S_pf_on = S_pf_row_bits[7];
                                4'd9:  S_pf_on = S_pf_row_bits[6];
                                4'd10: S_pf_on = S_pf_row_bits[5];
                                4'd11: S_pf_on = S_pf_row_bits[4];
                                4'd12: S_pf_on = S_pf_row_bits[3];
                                4'd13: S_pf_on = S_pf_row_bits[2];
                                4'd14: S_pf_on = S_pf_row_bits[1];
                                4'd15: S_pf_on = S_pf_row_bits[0];
                                default: S_pf_on = 1'b0;
                            endcase
                        end
                    end
                    else if((S_x_2d >= (DYN_OSD_X + 12'd52)) && (S_x_2d < (DYN_OSD_X + 12'd100)) && (S_y_2d >= (DYN_OSD_Y + 12'd4)) && (S_y_2d < (DYN_OSD_Y + 12'd20))) begin
                        if(S_x_2d < (DYN_OSD_X + 12'd68)) begin
                            S_pf_char = CHAR_C;
                            S_pf_local_x = S_x_2d - (DYN_OSD_X + 12'd52);
                        end
                        else if(S_x_2d < (DYN_OSD_X + 12'd84)) begin
                            S_pf_char = CHAR_N;
                            S_pf_local_x = S_x_2d - (DYN_OSD_X + 12'd68);
                        end
                        else begin
                            S_pf_char = CHAR_T;
                            S_pf_local_x = S_x_2d - (DYN_OSD_X + 12'd84);
                        end
                        S_pf_local_y = S_y_2d - (DYN_OSD_Y + 12'd4);
                        S_pf_col = S_pf_local_x[3:0];
                        S_pf_row = S_pf_local_y[3:0];
                        case(S_pf_col)
                            4'd0:  S_pf_on = S_pf_row_bits[15];
                            4'd1:  S_pf_on = S_pf_row_bits[14];
                            4'd2:  S_pf_on = S_pf_row_bits[13];
                            4'd3:  S_pf_on = S_pf_row_bits[12];
                            4'd4:  S_pf_on = S_pf_row_bits[11];
                            4'd5:  S_pf_on = S_pf_row_bits[10];
                            4'd6:  S_pf_on = S_pf_row_bits[9];
                            4'd7:  S_pf_on = S_pf_row_bits[8];
                            4'd8:  S_pf_on = S_pf_row_bits[7];
                            4'd9:  S_pf_on = S_pf_row_bits[6];
                            4'd10: S_pf_on = S_pf_row_bits[5];
                            4'd11: S_pf_on = S_pf_row_bits[4];
                            4'd12: S_pf_on = S_pf_row_bits[3];
                            4'd13: S_pf_on = S_pf_row_bits[2];
                            4'd14: S_pf_on = S_pf_row_bits[1];
                            4'd15: S_pf_on = S_pf_row_bits[0];
                            default: S_pf_on = 1'b0;
                        endcase
                    end
                    else if((S_x_2d >= (DYN_OSD_X + 12'd100)) && (S_x_2d < (DYN_OSD_X + 12'd108)) && (S_y_2d >= (DYN_OSD_Y + 12'd4)) && (S_y_2d < (DYN_OSD_Y + 12'd20))) begin
                        S_pf_char = CHAR_COLON;
                        S_pf_local_x = S_x_2d - (DYN_OSD_X + 12'd100);
                        S_pf_local_y = S_y_2d - (DYN_OSD_Y + 12'd4);
                        S_pf_col = {S_pf_local_x[2:0],1'b0};
                        S_pf_row = S_pf_local_y[3:0];
                        case(S_pf_col)
                            4'd0:  S_pf_on = S_pf_row_bits[15];
                            4'd1:  S_pf_on = S_pf_row_bits[14];
                            4'd2:  S_pf_on = S_pf_row_bits[13];
                            4'd3:  S_pf_on = S_pf_row_bits[12];
                            4'd4:  S_pf_on = S_pf_row_bits[11];
                            4'd5:  S_pf_on = S_pf_row_bits[10];
                            4'd6:  S_pf_on = S_pf_row_bits[9];
                            4'd7:  S_pf_on = S_pf_row_bits[8];
                            4'd8:  S_pf_on = S_pf_row_bits[7];
                            4'd9:  S_pf_on = S_pf_row_bits[6];
                            4'd10: S_pf_on = S_pf_row_bits[5];
                            4'd11: S_pf_on = S_pf_row_bits[4];
                            4'd12: S_pf_on = S_pf_row_bits[3];
                            4'd13: S_pf_on = S_pf_row_bits[2];
                            4'd14: S_pf_on = S_pf_row_bits[1];
                            4'd15: S_pf_on = S_pf_row_bits[0];
                            default: S_pf_on = 1'b0;
                        endcase
                    end

                    if(S_pf_on)
                        S_dyn_on = 1'b1;
                end

                // Decimal 6 digits starts at x=110. Each digit is 15x24.
                if((S_x_2d >= (DYN_OSD_X + 12'd110)) && (S_x_2d < (DYN_OSD_X + 12'd200))) begin
                    if(S_x_2d < (DYN_OSD_X + 12'd125)) begin
                        S_dyn_digit_idx = 4'd0;
                        S_dyn_digit_x = S_x_2d - (DYN_OSD_X + 12'd110);
                        S_dyn_dec_digit = S_cnt_d5;
                    end
                    else if(S_x_2d < (DYN_OSD_X + 12'd140)) begin
                        S_dyn_digit_idx = 4'd1;
                        S_dyn_digit_x = S_x_2d - (DYN_OSD_X + 12'd125);
                        S_dyn_dec_digit = S_cnt_d4;
                    end
                    else if(S_x_2d < (DYN_OSD_X + 12'd155)) begin
                        S_dyn_digit_idx = 4'd2;
                        S_dyn_digit_x = S_x_2d - (DYN_OSD_X + 12'd140);
                        S_dyn_dec_digit = S_cnt_d3;
                    end
                    else if(S_x_2d < (DYN_OSD_X + 12'd170)) begin
                        S_dyn_digit_idx = 4'd3;
                        S_dyn_digit_x = S_x_2d - (DYN_OSD_X + 12'd155);
                        S_dyn_dec_digit = S_cnt_d2;
                    end
                    else if(S_x_2d < (DYN_OSD_X + 12'd185)) begin
                        S_dyn_digit_idx = 4'd4;
                        S_dyn_digit_x = S_x_2d - (DYN_OSD_X + 12'd170);
                        S_dyn_dec_digit = S_cnt_d1;
                    end
                    else begin
                        S_dyn_digit_idx = 4'd5;
                        S_dyn_digit_x = S_x_2d - (DYN_OSD_X + 12'd185);
                        S_dyn_dec_digit = S_cnt_d0;
                    end

                    // Render decimal digits through unified 16x16 character library.
                    // Keep enlarged 15x24 box for readability.
                    if((S_dyn_digit_x < 12'd15) && (S_y_2d < (DYN_OSD_Y + 12'd24))) begin
                        S_pf_char = CHAR_0 + {4'd0, S_dyn_dec_digit};
                        S_pf_local_x = S_dyn_digit_x[4:0];
                        S_pf_local_y = S_dyn_y[4:0];
                        S_pf_col = S_pf_local_x[3:0];
                        S_pf_row = F_scale24_to16(S_pf_local_y);
                        case(S_pf_col)
                            4'd0:  S_pf_on = S_pf_row_bits[15];
                            4'd1:  S_pf_on = S_pf_row_bits[14];
                            4'd2:  S_pf_on = S_pf_row_bits[13];
                            4'd3:  S_pf_on = S_pf_row_bits[12];
                            4'd4:  S_pf_on = S_pf_row_bits[11];
                            4'd5:  S_pf_on = S_pf_row_bits[10];
                            4'd6:  S_pf_on = S_pf_row_bits[9];
                            4'd7:  S_pf_on = S_pf_row_bits[8];
                            4'd8:  S_pf_on = S_pf_row_bits[7];
                            4'd9:  S_pf_on = S_pf_row_bits[6];
                            4'd10: S_pf_on = S_pf_row_bits[5];
                            4'd11: S_pf_on = S_pf_row_bits[4];
                            4'd12: S_pf_on = S_pf_row_bits[3];
                            4'd13: S_pf_on = S_pf_row_bits[2];
                            4'd14: S_pf_on = S_pf_row_bits[1];
                            4'd15: S_pf_on = S_pf_row_bits[0];
                            default: S_pf_on = 1'b0;
                        endcase
                        if(S_pf_on)
                            S_dyn_on = 1'b1;
                    end
                end

                if(S_dyn_on) begin
                    S_osd_hit = 1'b1;
                    S_osd_color = COLOR_DYN_TEXT;
                end
            end

            // LAT is disjoint from logo/CNT. Its font address must not depend
            // on S_osd_hit, which itself depends on the shared font output.
            if((LATENCY_ENABLE!=0) && (S_x_2d >= LAT_OSD_X) && (S_x_2d < LAT_OSD_X + LAT_OSD_W) &&
               (S_y_2d >= LAT_OSD_Y) && (S_y_2d < LAT_OSD_Y + LAT_OSD_H)) begin
                S_lat_x = S_x_2d - LAT_OSD_X;
                S_lat_y = S_y_2d - LAT_OSD_Y;
                S_lat_col = S_lat_x[3:0];
                S_lat_row = S_lat_y[3:0];
                case(S_lat_x[7:4])
                    4'd0: S_lat_char = 8'h4c; // L
                    4'd1: S_lat_char = 8'h41; // A
                    4'd2: S_lat_char = CHAR_T;
                    4'd3: S_lat_char = CHAR_COLON;
                    4'd4: S_lat_char = S_latency_valid ?
                                      CHAR_0 + {4'd0,S_latency_bcd[11:8]} : 8'h58;
                    4'd5: S_lat_char = S_latency_valid ?
                                      CHAR_0 + {4'd0,S_latency_bcd[7:4]} : 8'h58;
                    4'd6: S_lat_char = S_latency_valid ?
                                      CHAR_0 + {4'd0,S_latency_bcd[3:0]} : 8'h58;
                    4'd7: S_lat_char = 8'h4d; // M (milliseconds)
                    4'd8: S_lat_char = 8'h53; // S
                    default: S_lat_char = 8'd0;
                endcase
                S_pf_char = S_lat_char;
                S_pf_row = S_lat_row;
                if(S_lat_row_bits[4'd15 - S_lat_col]) begin
                    S_osd_hit = 1'b1;
                    S_osd_color = (!S_latency_valid || S_latency_saturated) ?
                                  24'hff8000 : COLOR_DYN_TEXT;
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            O_hdmi_de        <= S_video_de_2d;
            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(diag_hit) O_hdmi_data <= diag_font[15-diag_dx[3:0]] ? 24'h00ffff : 24'h071520;
                else if(S_osd_hit)
                    O_hdmi_data <= S_osd_color;
                else if(TRAFFIC_ENABLE!=0 && S_traffic_hit)
                    O_hdmi_data <= S_traffic_color;
                else
                    O_hdmi_data <= READ_DATA_DELAY==2 ? I_video_rd_data :
                                   READ_DATA_DELAY==1 ? S_video_data_1d : S_video_data_2d;
            end
        end
    end

endmodule
