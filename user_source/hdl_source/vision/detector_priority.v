`timescale 1ns/1ps
// A secondary switch does not reset the first detector. Commit at camera SOF.
// Both-on from idle chooses red first; disabling the primary promotes the other.
module detector_priority(
 input I_clk,I_rst_n,I_start,input [3:0] I_flags,
 output [1:0] O_leader,output O_motion_primary,output O_epoch_changed);
 localparam NONE=0,RED=1,MOTION=2;
 reg [1:0] leader;reg [1:0] next_leader;
 always @*begin
  next_leader=leader;
  if(!I_flags[1]&&!I_flags[2])next_leader=NONE;
  else case(leader)
   RED:if(!I_flags[1])next_leader=MOTION;
   MOTION:if(!I_flags[2])next_leader=RED;
   default:next_leader=I_flags[1]?RED:MOTION;
  endcase
 end
 assign O_leader=I_start?next_leader:leader;
 assign O_motion_primary=O_leader==MOTION;
 assign O_epoch_changed=I_start&&(next_leader!=leader);
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)leader<=NONE;else if(I_start)leader<=next_leader;
endmodule
