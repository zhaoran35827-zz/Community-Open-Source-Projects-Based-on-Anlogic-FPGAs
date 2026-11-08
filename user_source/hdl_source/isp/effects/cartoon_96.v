`timescale 1ns/1ps
// Resource-first RGB888 cartoon stage. Four pixels/beat, first at [95:72].
// Data and all stream markers have ZERO clock latency, including bypass.
// COLOR_BITS=3: 8 levels/channel, expanded by bit repetition (pure wiring).
// Optional horizontal outlines keep just one 5-bit luminance sample.
module cartoon_96 #(
    parameter COLOR_BITS=3,
    parameter OUTLINE_ENABLE=0,
    parameter [4:0] EDGE_THRESHOLD=5'd4,
    parameter NEUTRAL_GUARD=1, ROUND_ENABLE=0, FIELD_FIX=0
)(
    input I_clk,I_rst_n,I_valid,I_frame_start,I_last,I_enable,
    input [95:0] I_data,
    output [95:0] O_data,
    output O_valid,O_frame_start,O_last
);
    assign O_valid=I_valid;
    assign O_frame_start=I_frame_start;
    assign O_last=I_last;
    wire [95:0] colors;
    wire [3:0] edge_mask;
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1)begin:g_color
        cartoon_color_24 #(.COLOR_BITS(COLOR_BITS),.NEUTRAL_GUARD(NEUTRAL_GUARD),.ROUND_ENABLE(ROUND_ENABLE),.FIELD_FIX(FIELD_FIX)) color (
            .I_rgb(I_data[95-lane*24 -:24]),
            .O_rgb(colors[95-lane*24 -:24])
        );
        assign O_data[95-lane*24 -:24]=!I_enable?I_data[95-lane*24 -:24]:
            ((I_valid && edge_mask[lane])?24'd0:colors[95-lane*24 -:24]);
    end endgenerate
    generate if(OUTLINE_ENABLE!=0)begin:g_outline
        wire [4:0] luma[0:3];
        reg [4:0] left_luma;
        reg left_valid;
        for(lane=0;lane<4;lane=lane+1)begin:g_luma
            wire [23:0] p=I_data[95-lane*24 -:24];
            // Approximate luminance / 8: R/32 + G/16 + B/32, max 29.
            assign luma[lane]={2'b0,p[23:21]}+{1'b0,p[15:12]}+{2'b0,p[7:5]};
            wire [4:0] previous;
            wire neighbor_valid;
            if(lane==0)begin:g_first
                assign previous=left_luma;
                assign neighbor_valid=left_valid && !I_frame_start;
            end else begin:g_next
                assign previous=luma[lane-1];
                assign neighbor_valid=1'b1;
            end
            wire [4:0] delta=(luma[lane]>=previous)?(luma[lane]-previous):(previous-luma[lane]);
            assign edge_mask[lane]=neighbor_valid && delta>=EDGE_THRESHOLD;
        end
        always @(posedge I_clk or negedge I_rst_n)begin
            if(!I_rst_n)begin left_luma<=0;left_valid<=0;end
            else begin
                // SOF may arrive before the first valid beat.
                if(I_frame_start)left_valid<=0;
                if(I_valid)begin left_luma<=luma[3];left_valid<=!I_last;end
            end
        end
    end else begin:g_no_outline
        assign edge_mask=4'b0000;
    end endgenerate
endmodule

// Independent color algorithm. No tables, RAM, DSP or registers.
// Near-gray channel noise straddling a bin boundary must not become purple
// or green. A shared green bin preserves neutral colors within +/-4 levels.
// Valid COLOR_BITS: 1..8. COLOR_BITS=8 is bit-exact identity.
module cartoon_color_24 #(parameter COLOR_BITS=3,parameter NEUTRAL_GUARD=1, ROUND_ENABLE=0, FIELD_FIX=0)(
    input [23:0] I_rgb,
    output [23:0] O_rgb
);
    wire [8:0] rg={1'b0,I_rgb[23:16]}-{1'b0,I_rgb[15:8]};
    wire [8:0] bg={1'b0,I_rgb[7:0]}-{1'b0,I_rgb[15:8]};
    wire neutral=FIELD_FIX!=0?((rg<=9'd12||rg>=9'd500)&&(bg<=9'd12||bg>=9'd500)):
        ((rg<=9'd4||rg>=9'd508)&&(bg<=9'd4||bg>=9'd508));
    wire [23:0] quantized;
    // Round to nearest expanded level instead of truncating dark colors.
    // q = round(channel * (2^COLOR_BITS-1) / 256), clamped by construction.
    // Constant shift/subtract, no divider, multiplier or image gain.
    wire [23:0] quant_source;
    genvar c;
    generate for(c=0;c<3;c=c+1)begin:g_round
        wire [7:0] value=I_rgb[c*8+:8];
        wire [16:0] scaled=({9'd0,value}<<COLOR_BITS)-value+17'd128;
        wire [7:0] code=scaled>>8;
        assign quant_source[c*8+:8]=(ROUND_ENABLE!=0 && COLOR_BITS<8)?code<<(8-COLOR_BITS):value;
    end endgenerate
    genvar channel,bit_index;
    generate for(channel=0;channel<3;channel=channel+1)begin:g_channel
      if(ROUND_ENABLE==2 && COLOR_BITS<8)begin:g_midpoint
        // Midpoint of each bin preserves dark detail with wiring and two
        // endpoint tests. Avoid twelve per-pixel arithmetic quantizers.
        wire [7:0] v=I_rgb[channel*8+:8];
        wire [7:0] middle=(v & (8'hff<<(8-COLOR_BITS))) | (8'h01<<(7-COLOR_BITS));
        assign quantized[channel*8+:8]=v==0?8'd0:v==255?8'd255:middle;
      end else begin:g_levels
        for(bit_index=0;bit_index<8;bit_index=bit_index+1)begin:g_bit
            assign quantized[channel*8+7-bit_index]=quant_source[channel*8+7-(bit_index%COLOR_BITS)];
        end
      end
    end endgenerate
    assign O_rgb=(NEUTRAL_GUARD!=0&&COLOR_BITS<8&&neutral)?{3{quantized[15:8]}}:quantized;
endmodule
