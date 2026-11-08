`timescale 1ns/1ps
// Four RGB888 pixels per beat: lane 0 occupies bits [23:0].
// Combinational debug stage: no added latency or handshake changes.
module color_segment_96 #(
    parameter DISPLAY_MODE = 1, // 0: original, 1: cyan overlay, 2: binary mask
    parameter [7:0] RED_MIN = 8'd80,
    parameter [7:0] RED_GREEN_MARGIN = 8'd30,
    parameter [7:0] RED_BLUE_MARGIN = 8'd30,
    parameter ADAPTIVE_LIGHT=0,
    // Reuse AWB raw-scene luminance; no pixel division or extra image storage.
    parameter [7:0] ADAPTIVE_MIN_RED=16,ADAPTIVE_MAX_RED=32,
    parameter WEAK_EXTENSION=0,
    parameter RED_HUE_GUARD=1,
    parameter [7:0] RED_HUE_SLOP=4,
    parameter STRICT_RED_SEED=0,
    parameter TIGHT_WEAK=0
) (
    input wire [95:0] I_tdata,
    input wire I_tvalid,
    output wire [95:0] O_tdata,
    output wire [3:0] O_red_mask,
    input [7:0] I_scene_luma,output [3:0] O_seed_mask
);
    // Scene mean sets only the noise floor, not an exposure gate. A dark
    // saturated target in a bright background must still be able to pass.
    wire [7:0] scene_floor=I_scene_luma>>3;
    wire [7:0] adaptive_floor=scene_floor<ADAPTIVE_MIN_RED?ADAPTIVE_MIN_RED:
        scene_floor>ADAPTIVE_MAX_RED?ADAPTIVE_MAX_RED:scene_floor;
    genvar lane;
    generate
        for (lane = 0; lane < 4; lane = lane + 1) begin : g_lane
            wire [7:0] red   = I_tdata[lane*24+16 +: 8];
            wire [7:0] green = I_tdata[lane*24+8  +: 8];
            wire [7:0] blue  = I_tdata[lane*24    +: 8];
            // Require R dominance in chroma, independently of global exposure.
            // Extension pixels retain the old shadow rule but cannot seed a body.
            wire [7:0] seed_margin=STRICT_RED_SEED!=0?((red>>2)+(red>>3)):(red>>2);
            wire [7:0] ratio_margin=seed_margin<4?8'd4:seed_margin;
            wire [7:0] floor_r=ADAPTIVE_LIGHT?adaptive_floor:RED_MIN;
            wire [7:0] margin_g=ADAPTIVE_LIGHT?ratio_margin:RED_GREEN_MARGIN;
            wire [7:0] margin_b=ADAPTIVE_LIGHT?ratio_margin:RED_BLUE_MARGIN;
            // Red-axis hue cone: |G-B| <= R-max(G,B)+slop. Reject
            // orange/brown and magenta that simple R dominance also accepts.
            // Ten-bit unsigned sums include the additive noise allowance.
            wire hue_ok=(!RED_HUE_GUARD) ||
                (({1'b0,green,1'b0} <= ({2'b0,red}+{2'b0,blue}+{2'b0,RED_HUE_SLOP})) &&
                 ({1'b0,blue,1'b0} <= ({2'b0,red}+{2'b0,green}+{2'b0,RED_HUE_SLOP})));
            // Strict seeds use a narrower red-axis cone. The older 2*G
            // test still admits saturated orange (255,128,0); use 3*G
            // versus R+2*B, and the symmetric test for red-violet.
            wire seed_hue_ok=STRICT_RED_SEED==0 || !RED_HUE_GUARD ||
                ((({1'b0,green,1'b0}+{2'b0,green}) <= ({2'b0,red}+{1'b0,blue,1'b0}+{2'b0,RED_HUE_SLOP})) &&
                 (({1'b0,blue,1'b0}+{2'b0,blue}) <= ({2'b0,red}+{1'b0,green,1'b0}+{2'b0,RED_HUE_SLOP})));
            // Fixed-threshold builds retain the exact original classifier.
            // Nine-bit sums prevent bright green/blue from wrapping at 255.
            // When R dominates, both hue inequalities reduce to one test
            // on chroma spread. Share the two subtractors with shadow extension.
            wire [7:0] hi=green>=blue?green:blue,lo=green>=blue?blue:green;
            wire [8:0] red_delta={1'b0,red}-{1'b0,hi};
            wire [8:0] chroma_delta={1'b0,hi}-{1'b0,lo};
            wire [9:0] hue_budget={1'b0,red_delta}+{2'b0,RED_HUE_SLOP};
            wire compact_seed=I_tvalid && red>=floor_r && red>=hi && red_delta>=ratio_margin &&
                (!RED_HUE_GUARD || {chroma_delta,1'b0}<=hue_budget);
            assign O_seed_mask[lane] = (ADAPTIVE_LIGHT!=0 && STRICT_RED_SEED!=0)?compact_seed:
                I_tvalid && (red >= floor_r) && (!ADAPTIVE_LIGHT || (hue_ok&&seed_hue_ok)) &&
                ({1'b0, red} >= ({1'b0, green} + {1'b0, margin_g})) &&
                ({1'b0, red} >= ({1'b0, blue} + {1'b0, margin_b}));
            // Weak shadow pixels are accepted only by a component that also
            // contains a strict red seed. No global white-balance widening.
            wire [7:0] weak_fraction=TIGHT_WEAK!=0?(red>>2):(red>>3);
            wire [7:0] weak_margin=weak_fraction<3?8'd3:weak_fraction;
            wire compact_weak=I_tvalid && red>=16 && red>=hi && red_delta>=weak_margin &&
                (!RED_HUE_GUARD || (TIGHT_WEAK!=0?{chroma_delta,1'b0}<=hue_budget:chroma_delta<=hue_budget));
            wire weak_red=(ADAPTIVE_LIGHT!=0 && STRICT_RED_SEED!=0)?compact_weak:
                I_tvalid && red>=16 && hue_ok &&
                {1'b0,red}>={1'b0,green}+{1'b0,weak_margin} &&
                {1'b0,red}>={1'b0,blue}+{1'b0,weak_margin};
            assign O_red_mask[lane]=O_seed_mask[lane] || (WEAK_EXTENSION!=0 && ADAPTIVE_LIGHT!=0 && weak_red);
            assign O_tdata[lane*24 +: 24] =
                (DISPLAY_MODE == 2) ? (O_red_mask[lane] ? 24'hffffff : 24'h000000) :
                (DISPLAY_MODE == 1 && O_red_mask[lane]) ? 24'h00ffff :
                I_tdata[lane*24 +: 24];
        end
    endgenerate
endmodule
