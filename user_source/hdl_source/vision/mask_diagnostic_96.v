`timescale 1ns/1ps
// Compare aligned raw and cleaned masks at the SAME pixel, not split scenes.
module mask_diagnostic_96 #(
    parameter ENABLE = 1
) (
    input wire I_valid,
    input wire [95:0] I_data,
    input wire [3:0] I_raw_mask,I_clean_mask,
    output wire [95:0] O_data
);
    genvar lane;
    generate for(lane=0;lane<4;lane=lane+1)begin:g_pixel
        assign O_data[lane*24 +:24]=(!ENABLE || !I_valid)?I_data[lane*24 +:24]:
            (I_raw_mask[lane] && I_clean_mask[lane])?24'h00ffff:
            I_raw_mask[lane]?24'hffff00:
            I_clean_mask[lane]?24'h0000ff:I_data[lane*24 +:24];
    end endgenerate
endmodule
