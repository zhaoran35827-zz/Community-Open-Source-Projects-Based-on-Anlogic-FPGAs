`timescale 1ns/1ps
// A/B alternate through one shared lane. All timing uses the independent
// 50 MHz board clock, so loss of the camera clock still produces fault red.
// Phase codes are also the display protocol: OFF,CLEAR,AG,AY,BG,BY,FAULT.
module traffic_controller #(
    parameter CLK_HZ=50000000,GREEN_MIN=6,GREEN_MAX=18,YELLOW_SEC=2,
    parameter CLEAR_SEC=2,WAIT_PRIORITY=25,BLOCK_SEC=30,
    parameter WATCHDOG_CYCLES=25000000
)(
    input I_clk,I_rst_n,I_enable,I_sample,
    input [2:0] I_count_a,I_count_b,
    input I_good,I_occupied,
    output reg [2:0] O_phase,
    output reg [4:0] O_seconds,
    output reg [2:0] O_count_a,O_count_b,
    output reg [7:0] O_wait_a,O_wait_b,
    output reg O_occupied,O_good,O_fault,
    output reg O_block_alarm,
    output reg [2:0] O_lamp_a,O_lamp_b // {red,yellow,green}, high=on
);
    localparam OFF=0,CLEAR=1,AG=2,AY=3,BG=4,BY=5,FAULT=6;
    localparam CW=(CLK_HZ<2)?1:$clog2(CLK_HZ);
    localparam WW=(WATCHDOG_CYCLES<2)?1:$clog2(WATCHDOG_CYCLES+1);
    reg [CW-1:0] second_div;
    reg [WW-1:0] watchdog;
    localparam CLW=(CLEAR_SEC<2)?1:$clog2(CLEAR_SEC+1);
    reg [CLW-1:0] clear_elapsed;
    localparam [63:0] BLOCK_LIMIT=64'd1*CLK_HZ*BLOCK_SEC;
    localparam BCW=(BLOCK_LIMIT<2)?1:$clog2(BLOCK_LIMIT);
    reg [BCW-1:0] occupied_cycles;
    reg seen,last_direction,armed;
    wire tick=(second_div==CLK_HZ-1);
    wire fresh=seen && watchdog<WATCHDOG_CYCLES;
    // Decisions must include an arriving snapshot on this very clock. In
    // particular, CLEAR must never release against yesterday's occupied bit.
    wire [2:0] count_a_now=I_sample?I_count_a:O_count_a;
    wire [2:0] count_b_now=I_sample?I_count_b:O_count_b;
    wire occupied_now=I_sample?(I_occupied || !I_good):O_occupied;
    wire good_now=I_sample?I_good:O_good;
    wire healthy=(I_sample || fresh) && good_now;
    wire force_a=count_a_now!=0 && O_wait_a>=WAIT_PRIORITY;
    wire force_b=count_b_now!=0 && O_wait_b>=WAIT_PRIORITY;
    wire choose_b=force_a && force_b ? (O_wait_b>O_wait_a ||
        (O_wait_b==O_wait_a && !last_direction)) : force_b ? 1'b1 : force_a ? 1'b0 :
        count_b_now>count_a_now ? 1'b1 : count_a_now>count_b_now ? 1'b0 : !last_direction;
    // Count consecutive HEALTHY occupancy from its first clock, across every
    // lamp phase. A camera error remains a different fault, never a false
    // blockage alarm. This is ROI occupancy, not individual-car stop tracking.
    wire block_timeout=BLOCK_LIMIT>0 && healthy && occupied_now && !O_fault &&
                       occupied_cycles>=BLOCK_LIMIT-1;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin occupied_cycles<=0;O_block_alarm<=0;end
        else if(!I_enable)begin occupied_cycles<=0;O_block_alarm<=0;end
        else if(!O_block_alarm)begin
            if(!healthy || !occupied_now || O_fault)occupied_cycles<=0;
            else if(block_timeout)O_block_alarm<=1;
            else occupied_cycles<=occupied_cycles+1'b1;
        end
    end
    function [4:0] green_time;
        input [2:0] n;reg [5:0] duration;
        begin
            case(n) 0,1:duration=GREEN_MIN;2:duration=GREEN_MIN+4;
                3:duration=GREEN_MIN+8;default:duration=GREEN_MAX;endcase
            green_time=duration>GREEN_MAX?GREEN_MAX:duration;
        end
    endfunction
    // Decode guarded outputs. Disable, fault, invalid state and lost video
    // always request both red, even before the state register is updated.
    always @* begin
        O_lamp_a=3'b100;O_lamp_b=3'b100;
        if(I_rst_n && I_enable && healthy && !O_fault && !O_block_alarm) begin
            case(O_phase)
                AG:O_lamp_a=3'b001;AY:O_lamp_a=3'b010;
                BG:O_lamp_b=3'b001;BY:O_lamp_b=3'b010;
                default:begin end
            endcase
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_phase<=OFF;O_seconds<=0;O_count_a<=0;O_count_b<=0;
            O_wait_a<=0;O_wait_b<=0;O_occupied<=1;O_good<=0;O_fault<=0;
            second_div<=0;watchdog<=WATCHDOG_CYCLES;seen<=0;
            clear_elapsed<=0;last_direction<=1;
            armed<=0;
        end else begin
            if(I_sample) begin
                watchdog<=0;seen<=1;
                O_count_a<=I_count_a;O_count_b<=I_count_b;
                O_good<=I_good;O_occupied<=I_occupied || !I_good;
            end else if(watchdog<WATCHDOG_CYCLES) watchdog<=watchdog+1'b1;
            if(!fresh && !I_sample) begin O_good<=0;O_occupied<=1;end
            if(!I_enable) begin
                O_phase<=OFF;O_seconds<=0;O_wait_a<=0;O_wait_b<=0;
                O_fault<=0;second_div<=0;clear_elapsed<=0;
                armed<=0;
            end else if(block_timeout || O_block_alarm) begin
                O_phase<=FAULT;O_fault<=1;O_seconds<=0;second_div<=0;clear_elapsed<=0;
            end else if(O_phase==OFF) begin
                O_phase<=CLEAR;O_seconds<=0;second_div<=0;
                clear_elapsed<=0;O_fault<=0;
            end else if(!healthy || O_phase>FAULT) begin
                O_phase<=(armed || O_phase>FAULT)?FAULT:CLEAR;
                O_fault<=armed || O_phase>FAULT;O_seconds<=0;second_div<=0;
                clear_elapsed<=0;
            end else if(O_phase==FAULT) begin
                // Require external traffic switch OFF -> ON after a fault.
                O_fault<=1;O_seconds<=0;second_div<=0;
            end else begin
                armed<=1;
                if(tick) second_div<=0;else second_div<=second_div+1'b1;
                // Start a new wait after every service, including a continuously
                // nonempty queue. Do not require the ROI to become empty first.
                if(count_a_now==0 || O_phase==AG) O_wait_a<=0;
                else if(tick && O_wait_a!=255) O_wait_a<=O_wait_a+1'b1;
                if(count_b_now==0 || O_phase==BG) O_wait_b<=0;
                else if(tick && O_wait_b!=255) O_wait_b<=O_wait_b+1'b1;
                case(O_phase)
                    CLEAR:begin
                        if(tick) begin
                            if(clear_elapsed<CLEAR_SEC) clear_elapsed<=clear_elapsed+1'b1;
                        end
                        if(clear_elapsed>=CLEAR_SEC && !occupied_now &&
                           (count_a_now!=0 || count_b_now!=0)) begin
                            if(count_a_now==0 || (count_b_now!=0 && choose_b)) begin
                                O_phase<=BG;O_seconds<=green_time(count_b_now);
                                O_wait_b<=0;last_direction<=1;
                            end else begin
                                O_phase<=AG;O_seconds<=green_time(count_a_now);
                                O_wait_a<=0;last_direction<=0;
                            end
                            second_div<=0;clear_elapsed<=0;
                        end
                    end
                    AG:if(tick) begin
                        if(O_seconds<=1) begin O_phase<=AY;O_seconds<=YELLOW_SEC;end
                        else O_seconds<=O_seconds-1'b1;
                    end
                    BG:if(tick) begin
                        if(O_seconds<=1) begin O_phase<=BY;O_seconds<=YELLOW_SEC;end
                        else O_seconds<=O_seconds-1'b1;
                    end
                    AY,BY:if(tick) begin
                        if(O_seconds<=1) begin
                            O_phase<=CLEAR;O_seconds<=0;clear_elapsed<=0;
                        end else O_seconds<=O_seconds-1'b1;
                    end
                    default:begin O_phase<=FAULT;O_fault<=1;O_seconds<=0;end
                endcase
            end
        end
    end
endmodule
