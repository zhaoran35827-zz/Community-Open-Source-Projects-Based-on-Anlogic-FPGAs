`timescale 1ns/1ps
// Switch wiring is deliberately outside this project. Inputs are same-domain
// flags; commit all features together at SOF, before the first valid pixel.
module frame_feature_flags #(parameter [2:0] DEFAULT_FLAGS=3'b111)(
    input I_clk,I_rst_n,I_start,
    input [2:0] I_requested,
    output [2:0] O_flags
);
    reg [2:0] active;
    assign O_flags=I_start?I_requested:active;
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n)active<=DEFAULT_FLAGS;
        else if(I_start)active<=I_requested;
endmodule
