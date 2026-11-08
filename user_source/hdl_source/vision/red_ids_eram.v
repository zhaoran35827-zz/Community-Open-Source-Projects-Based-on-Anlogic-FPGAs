`timescale 1ns/1ps
// Red-ID specialisation of target_tracks_96(DB_META=1,LIVE_BOX_INPUTS=1,
// BOX_DB=1, observed=all1, hints/forget=0). Same greedy matching, allocation,
// age, 1..99 ID and BCD rules; geometry and next geometry live in one ERAM.
// Input coordinates must remain stable until O_ready. Native CCL holds them
// for a complete camera frame. No main-video handshake depends on this worker.
module red_ids_eram #(parameter TARGETS=6,CB=3,IB=$clog2(TARGETS),MAX_DISTANCE=128,MAX_MISSED=8)(
 input I_clk,I_rst_n,I_clear,I_start,input [CB-1:0] I_count,
 input [TARGETS*16-1:0] I_x0,I_x1,I_y0,I_y1,input [4:0] I_reason,
 output reg [TARGETS*8-1:0] O_ids,output reg O_ready,
 output [TARGETS-1:0] O_active,output [TARGETS*7-1:0] O_tags);
 localparam IDLE=0,LOAD=1,WRITE0=2,WRITE1=3,LOAD_NEXT=4,READ0=5,READ0_Q=6,READ1=7,READ1_Q=8,
  PREP=9,SCAN_D=10,SCAN_D_DONE=11,SCAN_T=12,SCAN_CHECK=13,SCAN_NEXT_T=14,SCAN_NEXT_D=15,MATCH=16,
  ALLOC=17,IDCHECK=18,ALLOC_NEXT=19,BUILD=20,BUILD_RECORD=21,BUILD_NEXT=22,COMMIT=23;
 reg [4:0] state,return_state;reg start_d,page,read_track,mapped_work;
 reg [CB-1:0] count;reg [IB-1:0] index,scan_d,scan_t,round,best_d,best_t,pick,mapped_index;
 reg [1:0] read_set,write_set;reg [IB-1:0] read_index,write_index;
 reg [41:0] det_record,track_record,write_record;
 reg [TARGETS-1:0] active,next_active,det_valid,det_used,track_used;
 reg [6:0] tid[0:TARGETS-1],next_tid[0:TARGETS-1],did[0:TARGETS-1],next_id;
 reg [3:0] age[0:TARGETS-1],next_age[0:TARGETS-1];
 reg [IB-1:0] destination[0:TARGETS-1];
 reg [TARGETS*8-1:0] display_ids;
 reg found;reg [11:0] best_cost;
 wire sof=I_start&&!start_d;
 wire rd=state==READ0||state==READ1,wr=state==WRITE0||state==WRITE1;
 wire [5:0] address=wr?{write_set,write_index,1'b0}+(state==WRITE1):{read_set,read_index,1'b0}+(state==READ1);
 wire [20:0] q;wire [20:0] wd=state==WRITE1?write_record[41:21]:write_record[20:0];
 motion_object_ram #(.WIDTH(21),.WORDS(64),.AB(6),.TWO_PORTS(0)) records(
  .I_clk(I_clk),.A_enable(rd||wr),.A_write(wr),.A_address(address),.A_data(wd),.A_q(q),
  .B_enable(1'b0),.B_write(1'b0),.B_address(6'd0),.B_data(21'd0),.B_q());
 assign O_active=active;
 genvar t;generate for(t=0;t<TARGETS;t=t+1)begin:g_tags assign O_tags[t*7+:7]=tid[t];end endgenerate
 wire [15:0] lx0=I_x0[index*16+:16],lx1=I_x1[index*16+:16],ly0=I_y0[index*16+:16],ly1=I_y1[index*16+:16];
 wire [11:0] dsx={1'b0,det_record[10:0]}+{1'b0,det_record[21:11]},tsx={1'b0,track_record[10:0]}+{1'b0,track_record[21:11]};
 wire [10:0] dsy={1'b0,det_record[31:22]}+{1'b0,det_record[41:32]},tsy={1'b0,track_record[31:22]}+{1'b0,track_record[41:32]};
 wire [10:0] dw=det_record[21:11]-det_record[10:0],tw=track_record[21:11]-track_record[10:0];
 wire [9:0] dh=det_record[41:32]-det_record[31:22],th=track_record[41:32]-track_record[31:22];
 wire [10:0] ax=dsx[11:1]>=tsx[11:1]?dsx[11:1]-tsx[11:1]:tsx[11:1]-dsx[11:1];
 wire [9:0] ay=dsy[10:1]>=tsy[10:1]?dsy[10:1]-tsy[10:1]:tsy[10:1]-dsy[10:1];
 wire [10:0] aw=dw>=tw?dw-tw:tw-dw;wire [9:0] ah=dh>=th?dh-th:th-dh;
 wire [11:0] distance={1'b0,ax}+{2'b0,ay};
 wire eligible=distance<=MAX_DISTANCE&&aw<=({1'b0,tw}>>1)+12'd8&&ah<=({1'b0,th}>>1)+11'd8;
 reg free_found,id_collision,mapped;reg [IB-1:0] free_slot,mapped_det;reg [3:0] oldest;integer j;
 always @*begin
  free_found=0;free_slot=0;oldest=0;mapped=0;mapped_det=0;id_collision=0;
  for(j=0;j<TARGETS;j=j+1)if(!free_found&&!active[j]&&!track_used[j])begin free_found=1;free_slot=j;end
  if(!free_found)for(j=0;j<TARGETS;j=j+1)if(!track_used[j]&&(!free_found||age[j]>oldest))begin free_found=1;free_slot=j;oldest=age[j];end
  for(j=0;j<TARGETS;j=j+1)begin
   if(active[j]&&tid[j]==next_id)id_collision=1;
   if(det_used[j]&&did[j]==next_id)id_collision=1;
   if(det_used[j]&&destination[j]==index)begin mapped=1;mapped_det=j;end
  end
 end
 function [7:0] decimal_id;input [6:0] value;integer k;reg [7:0] b;
 begin b=0;for(k=6;k>=0;k=k-1)begin if(b[3:0]>=5)b[3:0]=b[3:0]+3;if(b[7:4]>=5)b[7:4]=b[7:4]+3;b={b[6:0],value[k]};end decimal_id=b;end endfunction
 integer n;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  state<=IDLE;return_state<=0;start_d<=0;page<=0;read_track<=0;mapped_work<=0;count<=0;index<=0;scan_d<=0;scan_t<=0;round<=0;best_d<=0;best_t<=0;pick<=0;mapped_index<=0;
  read_set<=0;write_set<=0;read_index<=0;write_index<=0;det_record<=0;track_record<=0;write_record<=0;
  active<=0;next_active<=0;det_valid<=0;det_used<=0;track_used<=0;next_id<=1;display_ids<=0;O_ids<=0;O_ready<=1;found<=0;best_cost<=4095;
  for(n=0;n<TARGETS;n=n+1)begin tid[n]<=0;next_tid[n]<=0;did[n]<=0;age[n]<=0;next_age[n]<=0;destination[n]<=0;end
 end else if(I_clear)begin
  state<=IDLE;start_d<=0;active<=0;next_active<=0;count<=0;det_used<=0;det_valid<=0;track_used<=0;next_id<=1;O_ids<=0;O_ready<=1;
 end else begin
  start_d<=I_start;
  if(sof)begin
   state<=LOAD;index<=0;count<=I_count;det_valid<=0;det_used<=0;track_used<=0;display_ids<=0;O_ready<=0;
   for(n=0;n<TARGETS;n=n+1)did[n]<=0;
   if(I_reason!=0)begin state<=IDLE;O_ids<=0;O_ready<=1;end
   else if(I_count==0)begin state<=IDLE;active<=0;next_id<=1;O_ids<=0;O_ready<=1;end
  end else case(state)
   IDLE:;
   LOAD:begin
    det_valid[index]<=index<count&&lx0<=lx1&&ly0<=ly1&&lx1<1280&&ly1<720&&lx1-lx0>=3&&ly1-ly0>=3;
    write_record<={ly1[9:0],ly0[9:0],lx1[10:0],lx0[10:0]};write_set<=2;write_index<=index;return_state<=LOAD_NEXT;state<=WRITE0;
   end
   WRITE0:state<=WRITE1;
   WRITE1:state<=return_state;
   LOAD_NEXT:if(index==TARGETS-1)begin round<=0;state<=PREP;end else begin index<=index+1'b1;state<=LOAD;end
   READ0:state<=READ0_Q;
   READ0_Q:begin if(read_track)track_record[20:0]<=q;else det_record[20:0]<=q;state<=READ1;end
   READ1:state<=READ1_Q;
   READ1_Q:begin if(read_track)track_record[41:21]<=q;else det_record[41:21]<=q;state<=return_state;end
   PREP:begin scan_d<=0;found<=0;best_cost<=4095;state<=SCAN_D;end
   SCAN_D:if(!det_valid[scan_d]||det_used[scan_d])state<=SCAN_NEXT_D;
   else begin read_set<=2;read_index<=scan_d;read_track<=0;return_state<=SCAN_D_DONE;state<=READ0;end
   SCAN_D_DONE:begin scan_t<=0;state<=SCAN_T;end
   SCAN_T:if(!active[scan_t]||track_used[scan_t])state<=SCAN_NEXT_T;
   else begin read_set<={1'b0,page};read_index<=scan_t;read_track<=1;return_state<=SCAN_CHECK;state<=READ0;end
   SCAN_CHECK:begin
    if(eligible&&distance<best_cost)begin found<=1;best_cost<=distance;best_d<=scan_d;best_t<=scan_t;end state<=SCAN_NEXT_T;
   end
   SCAN_NEXT_T:if(scan_t==TARGETS-1)state<=SCAN_NEXT_D;else begin scan_t<=scan_t+1'b1;state<=SCAN_T;end
   SCAN_NEXT_D:if(scan_d==TARGETS-1)state<=MATCH;else begin scan_d<=scan_d+1'b1;state<=SCAN_D;end
   MATCH:begin
    if(found)begin det_used[best_d]<=1;track_used[best_t]<=1;did[best_d]<=tid[best_t];destination[best_d]<=best_t;end
    if(!found||round==TARGETS-1)begin index<=0;state<=ALLOC;end else begin round<=round+1'b1;state<=PREP;end
   end
   ALLOC:if(det_valid[index]&&!det_used[index]&&free_found)begin pick<=free_slot;state<=IDCHECK;end else state<=ALLOC_NEXT;
   IDCHECK:if(id_collision)next_id<=next_id==99?1:next_id+1'b1;
   else begin did[index]<=next_id;destination[index]<=pick;det_used[index]<=1;track_used[pick]<=1;next_id<=next_id==99?1:next_id+1'b1;state<=ALLOC_NEXT;end
   ALLOC_NEXT:if(index==TARGETS-1)begin index<=0;state<=BUILD;end else begin index<=index+1'b1;state<=ALLOC;end
   BUILD:begin
    mapped_work<=mapped;mapped_index<=mapped_det;read_set<=mapped?2:{1'b0,page};read_index<=mapped?mapped_det:index;read_track<=!mapped;return_state<=BUILD_RECORD;state<=READ0;
   end
   BUILD_RECORD:begin
    if(mapped_work)begin write_record<=det_record;next_tid[index]<=did[mapped_index];next_age[index]<=0;next_active[index]<=1;end
    else begin write_record<=track_record;next_tid[index]<=tid[index];next_age[index]<=age[index]+1'b1;next_active[index]<=active[index]&&age[index]<MAX_MISSED-1;end
    display_ids[index*8+:8]<=det_valid[index]&&det_used[index]?decimal_id(did[index]):0;
    write_set<={1'b0,!page};write_index<=index;return_state<=BUILD_NEXT;state<=WRITE0;
   end
   BUILD_NEXT:if(index==TARGETS-1)state<=COMMIT;else begin index<=index+1'b1;state<=BUILD;end
   COMMIT:begin
    for(n=0;n<TARGETS;n=n+1)begin tid[n]<=next_tid[n];age[n]<=next_age[n];end
    active<=next_active;page<=!page;O_ids<=display_ids;O_ready<=1;state<=IDLE;
   end
  endcase
 end
endmodule
