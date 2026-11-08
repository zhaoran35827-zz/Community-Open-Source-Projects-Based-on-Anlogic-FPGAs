`timescale 1ns/1ps
// Fixed-size, continuously sampled video stream, four pixels per valid beat.
// No added video latency. Like the existing AWB/packer, advances on valid,
// not ready; this is not a standalone backpressure-capable AXI component.
// The displayed box is the completed previous frame's result.
module red_bbox_96 #(
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter MIN_AREA = 100,
    parameter BORDER_WIDTH = 2,
    parameter SMOOTH_ENABLE = 1,
    parameter JITTER_PIXELS = 3,
    parameter SNAP_PIXELS = 32,
    parameter MIN_BOX_AREA=0
) (
    input wire I_clk,
    input wire I_rst_n,
    input wire I_frame_start,
    input wire I_valid,
    input wire [95:0] I_data,
    input wire [3:0] I_mask,
    output wire [95:0] O_data,
    output reg O_box_valid,
    output reg [15:0] O_xmin, O_xmax, O_ymin, O_ymax,
    output reg [31:0] O_area,
    output wire [15:0] O_pixel_x,
    output wire [15:0] O_pixel_y
);
    reg [15:0] x_base, y;
    reg frame_start_d;
    reg [15:0] xmin, xmax, ymin, ymax;
    reg [31:0] area;
    wire new_frame = I_frame_start && !frame_start_d;
    wire [15:0] current_x = new_frame ? 16'd0 : x_base;
    wire [15:0] current_y = new_frame ? 16'd0 : y;
    assign O_pixel_x = current_x;
    assign O_pixel_y = current_y;
    reg [15:0] next_xmin, next_xmax, next_ymin, next_ymax;
    reg [31:0] next_area;
    wire [16:0] measured_width={1'b0,next_xmax}-{1'b0,next_xmin}+17'd1;
    wire [16:0] measured_height={1'b0,next_ymax}-{1'b0,next_ymin}+17'd1;
    wire [33:0] measured_box=measured_width*measured_height;
    wire area_ok=next_area>=MIN_AREA&&next_area!=0&&measured_box>=MIN_BOX_AREA;
    integer offset;
    // Per-frame edge hysteresis plus 1/4 tracking step. Integer-only logic.
    // A large edge jump snaps immediately to avoid prolonged tracking lag.
    function [15:0] stable_edge;
        input [15:0] old_value;
        input [15:0] measured;
        reg [16:0] distance;
        reg [16:0] step;
        begin
            distance = measured >= old_value ?
                {1'b0,measured}-{1'b0,old_value} : {1'b0,old_value}-{1'b0,measured};
            step = (distance + 17'd3) >> 2;
            if (!SMOOTH_ENABLE || distance >= SNAP_PIXELS) stable_edge = measured;
            else if (distance <= JITTER_PIXELS) stable_edge = old_value;
            else if (measured > old_value) stable_edge = old_value + step;
            else stable_edge = old_value - step;
        end
    endfunction
    always @* begin
        next_xmin = new_frame ? IMG_WIDTH-1 : xmin;
        next_xmax = new_frame ? 16'd0 : xmax;
        next_ymin = new_frame ? IMG_HEIGHT-1 : ymin;
        next_ymax = new_frame ? 16'd0 : ymax;
        next_area = new_frame ? 32'd0 : area;
        // Packer/video_out transmit the most significant RGB pixel first.
        // color_segment_96 names its least-significant lane mask[0].
        for (offset = 0; offset < 4; offset = offset + 1) begin
            if (I_valid && I_mask[3-offset]) begin
                next_area = next_area + 1'b1;
                if (current_x + offset < next_xmin) next_xmin = current_x + offset;
                if (current_x + offset > next_xmax) next_xmax = current_x + offset;
                if (current_y < next_ymin) next_ymin = current_y;
                if (current_y > next_ymax) next_ymax = current_y;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            frame_start_d <= 0;
            x_base <= 0; y <= 0;
            xmin <= IMG_WIDTH-1; xmax <= 0;
            ymin <= IMG_HEIGHT-1; ymax <= 0; area <= 0;
            O_box_valid <= 0;
            O_xmin <= 0; O_xmax <= 0; O_ymin <= 0; O_ymax <= 0; O_area <= 0;
        end else begin
            frame_start_d <= I_frame_start;
            if (new_frame || I_valid) begin
                xmin <= next_xmin; xmax <= next_xmax;
                ymin <= next_ymin; ymax <= next_ymax; area <= next_area;
            end
            if (new_frame) begin
                x_base <= 0; y <= 0;
            end
            if (I_valid) begin
                if (current_x == IMG_WIDTH-4) begin
                    x_base <= 0;
                    if (current_y == IMG_HEIGHT-1) begin
                        y <= 0;
                        O_box_valid <= area_ok;
                        // First acquisition snaps. No-target frames are not held.
                        if (O_box_valid && area_ok) begin
                            O_xmin <= stable_edge(O_xmin,next_xmin);
                            O_xmax <= stable_edge(O_xmax,next_xmax);
                            O_ymin <= stable_edge(O_ymin,next_ymin);
                            O_ymax <= stable_edge(O_ymax,next_ymax);
                        end else begin
                            O_xmin <= next_xmin; O_xmax <= next_xmax;
                            O_ymin <= next_ymin; O_ymax <= next_ymax;
                        end
                        O_area <= next_area;
                        xmin <= IMG_WIDTH-1; xmax <= 0;
                        ymin <= IMG_HEIGHT-1; ymax <= 0; area <= 0;
                    end else y <= current_y + 1'b1;
                end else begin
                    x_base <= current_x + 16'd4;
                    y <= current_y;
                end
            end
        end
    end

    genvar p;
    generate for (p = 0; p < 4; p = p + 1) begin : g_draw
        wire [15:0] px = current_x + p;
        wire inside_box = (px >= O_xmin) && (px <= O_xmax) &&
                          (current_y >= O_ymin) && (current_y <= O_ymax);
        wire border = inside_box &&
            ((px - O_xmin < BORDER_WIDTH) || (O_xmax - px < BORDER_WIDTH) ||
             (current_y - O_ymin < BORDER_WIDTH) || (O_ymax - current_y < BORDER_WIDTH));
        assign O_data[(3-p)*24 +: 24] = (I_valid && O_box_valid && border) ?
            24'h00ff00 : I_data[(3-p)*24 +: 24];
    end endgenerate
endmodule
