`timescale 1ns/1ps
// Manual position-servo control, independent of image processing. All state is
// clocked by I_clk; tick_us is a clock enable, never a generated clock.
// Keys are active low: {down, up, right, left}. A key must be observed pressed
// at two consecutive frame boundaries (20 ms apart by default). The current
// synchronized release/opposite key also vetoes motion immediately at a step.
// Model/installation-specific pulse limits and direction are parameters.
module manual_gimbal #(
    parameter integer CLK_HZ=50000000,
    parameter integer FRAME_US=20000,
    parameter integer PAN_MIN_US=1100,PAN_MAX_US=1900,
    parameter integer TILT_MIN_US=800,TILT_MAX_US=2400,
    parameter integer CENTER_US=1500,STEP_US=10,
    parameter integer STEP_FRAMES=2,
    parameter PAN_REVERSE=0,TILT_REVERSE=0
)(
    input I_clk,I_rst_n,I_lock,
    input [3:0] I_key_n,
    output reg O_pan_pwm,O_tilt_pwm
);
    localparam integer US_DIV=CLK_HZ/1000000;
    localparam integer DW=(US_DIV<2)?1:$clog2(US_DIV);
    localparam integer FW=(FRAME_US<2)?1:$clog2(FRAME_US);
    localparam integer MAX_US=(PAN_MAX_US>TILT_MAX_US)?PAN_MAX_US:TILT_MAX_US;
    localparam integer PW=(MAX_US<2)?1:$clog2(MAX_US);
    localparam integer SW=(STEP_FRAMES<2)?1:$clog2(STEP_FRAMES);
    // Store pulse end index, i.e. width_us-1. Equality comparison can then
    // terminate the pulse without a variable subtractor or wide comparator.
    reg [PW-1:0] pan_end,tilt_end;
    reg [DW-1:0] us_div;
    reg [FW-1:0] frame_us;
    reg [SW-1:0] step_frame;
    (* async_reg="true" *) reg [3:0] key_meta,key_sync;
    reg [3:0] previous_key_n;
    reg sample_seen,manual_armed;
    wire tick_us=us_div==US_DIV-1;
    wire frame_end=frame_us==FRAME_US-1;
    wire step_due=step_frame==STEP_FRAMES-1;
    wire [3:0] pressed=(~key_sync)&(~previous_key_n);
    wire left=pressed[0] && key_sync[1];
    wire right=pressed[1] && key_sync[0];
    wire up=pressed[2] && key_sync[3];
    wire down=pressed[3] && key_sync[2];
    wire pan_inc=PAN_REVERSE?left:right;
    wire pan_dec=PAN_REVERSE?right:left;
    wire tilt_inc=TILT_REVERSE?down:up;
    wire tilt_dec=TILT_REVERSE?up:down;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            us_div<=0;frame_us<=FRAME_US-1;step_frame<=0;
            pan_end<=CENTER_US-1;tilt_end<=CENTER_US-1;
            O_pan_pwm<=0;O_tilt_pwm<=0;
            key_meta<=4'b1111;key_sync<=4'b1111;
            previous_key_n<=4'b1111;sample_seen<=0;manual_armed<=0;
        end else begin
            key_meta<=I_key_n;key_sync<=key_meta;
            // Lock only freezes the command. Keep sending complete pulses to
            // hold the camera position; never gate the clock or truncate PWM.
            if(I_lock) manual_armed<=0;
            if(tick_us) begin
                us_div<=0;
                if(frame_end) begin
                    frame_us<=0;O_pan_pwm<=1;O_tilt_pwm<=1;
                    previous_key_n<=key_sync;sample_seen<=1;
                    if(step_due) step_frame<=0;else step_frame<=step_frame+1'b1;
                    if(!I_lock && sample_seen && (&key_sync) && (&previous_key_n))
                        manual_armed<=1;
                    // No queued target exists: only the frame boundary can
                    // change an axis position, so the active pulse is coherent.
                    if(!I_lock && manual_armed && step_due) begin
                        if(pan_inc) begin
                            if(pan_end>=PAN_MAX_US-1-STEP_US) pan_end<=PAN_MAX_US-1;
                            else pan_end<=pan_end+STEP_US;
                        end else if(pan_dec) begin
                            if(pan_end<=PAN_MIN_US-1+STEP_US) pan_end<=PAN_MIN_US-1;
                            else pan_end<=pan_end-STEP_US;
                        end
                        if(tilt_inc) begin
                            if(tilt_end>=TILT_MAX_US-1-STEP_US) tilt_end<=TILT_MAX_US-1;
                            else tilt_end<=tilt_end+STEP_US;
                        end else if(tilt_dec) begin
                            if(tilt_end<=TILT_MIN_US-1+STEP_US) tilt_end<=TILT_MIN_US-1;
                            else tilt_end<=tilt_end-STEP_US;
                        end
                    end
                end else begin
                    frame_us<=frame_us+1'b1;
                    if(frame_us==pan_end) O_pan_pwm<=0;
                    if(frame_us==tilt_end) O_tilt_pwm<=0;
                end
            end else us_div<=us_div+1'b1;
        end
    end
endmodule
