`timescale 1ns/1ps
//16 provisional templates,32 complete regions, six final merged observations.
// ERAM holds descriptors/proposals/temporary clusters. The one patch matcher
// is shared with camera compensation; the existing six-ID tracker is downstream.
module motion_object_tracks #(parameter WIDTH=640,HEIGHT=360,TARGETS=6,CB=3,SINGLE_OUTPUT=0,RECORDS=16,ROBUST_MATCH=0)(
 input I_clk,I_rst_n,I_begin,I_abort,I_assist,input [7:0] I_frame,
 input [47:0] I_camera_x,I_camera_y,
 input G_valid,G_done,G_overflow,output G_ready,
 input [7:0] G_x0,G_x1,G_y0,G_y1,input [14:0] G_area,
 output reg O_busy,O_done,O_limited,output reg [7:0] O_frame,
 output reg [CB-1:0] O_count,O_moving_count,
 output reg [TARGETS*16-1:0] O_x0,O_x1,O_y0,O_y1,
 output wire [TARGETS*16-1:0] O_mx0,O_mx1,O_my0,O_my1,
 output reg [15:0] O_bx0,O_bx1,O_by0,O_by1,
 output reg [TARGETS-1:0] O_observed,
 output reg P_begin,P_learn,output [9:0] P_template,
 output signed [11:0] P_x0,P_x1,P_y0,P_y1,
 output reg signed [7:0] P_cx,P_cy,P_tie_x,P_tie_y,
 input P_done,P_quality,input signed [7:0] P_dx,P_dy,input [15:0] P_score,input [15:0] P_reference_score);
 localparam CLEAR=0,IDLE=1,COLLECT=2,PROP_WRITE0=3,PROP_WRITE1=4,PROP_MIN=5,
  READ_REQ=6,READ_Q=7,WRITE=8,REC_START=9,REC_LOADED=10,
  ASSOC_REQ0=11,ASSOC_Q0=12,ASSOC_REQ1=13,ASSOC_Q1=14,ASSOC_NEXT=15,
  SEARCH_START=16,SEARCH_WAIT=17,SEARCH_APPLY=18,REFRESH_START=19,REFRESH_WAIT=20,
  REC_SAVE=21,REC_CLUSTER=22,REC_NEXT=23,BIRTH_START=24,BIRTH_Q0=25,BIRTH_Q1=26,
  DUP_START=27,DUP_CHECK=28,DUP_NEXT=29,BIRTH_LEARN=30,BIRTH_WAIT=31,BIRTH_APPLY=32,BIRTH_NEXT=33,
  MERGE_START=34,MERGE_A=35,MERGE_A_DONE=36,MERGE_B=37,MERGE_TEST=38,MERGE_SAVE_A=39,MERGE_NEXT=40,
  EMIT_START=41,EMIT_GROUP=42,EMIT_GROUP_DONE=43,OWNER_NEXT=44,OWNER_META=45,OWNER_WRITE=46,EMIT_NEXT=47,FINISH=48,VECTOR_WAIT=49,BIRTH_SELECT=50,BIRTH_RANK=51,PROP_MIN_FETCH=52,BIRTH_RANK_FETCH=53;
 reg [5:0] state,return_state;reg assist,regions_done,limited;
 reg [3:0] slot,record_index,dup_index,birth_slot,merge_i,merge_j,group_index,member_index;
 reg [2:0] word_index,word_last;reg [3:0] word_base;reg cache_high;
 reg [35:0] c0,c2,c4,c6;reg [23:0] c1,c5;reg [15:0] c3,c7;wire [35:0] record_q;
 reg [35:0] cache_read_data;wire [2:0] cache_address={cache_high,2'b00}+word_index;
 always @*case(cache_address)
  0:cache_read_data=c0;1:cache_read_data=c1;2:cache_read_data=c2;3:cache_read_data=c3;
  4:cache_read_data=c4;5:cache_read_data=c5;6:cache_read_data=c6;7:cache_read_data=c7;
 endcase
 reg [RECORDS-1:0] valid_mask,cluster_mask;reg [7:0] next_uid;
 reg [5:0] prop_count,prop_index,min_index,selected_prop;
 reg [31:0] prop_used;reg [14:0] minimum_area;
 reg [31:0] birth_seen;
 reg [35:0] proposed_box,proposed_stats,prop_box;wire [35:0] prop_q;
 reg [14:0] best_distance;reg [5:0] best_prop;
 reg [31:0] best_box;
 reg signed [11:0] cam_x,cam_y;
 reg vector_begin;wire vector_done;wire signed [11:0] vector_x,vector_y;
 reg duplicate,merge_changed,uid_conflict;
 reg [35:0] group_x,group_y,group_meta;reg [15:0] group_members;
 reg [CB-1:0] count,moving_count;
 assign P_template=10'd384+({6'd0,slot}<<4)+({6'd0,slot}<<2);
 assign P_x0=c1[11:0];assign P_x1=c1[23:12];
 assign P_y0=c2[11:0];assign P_y1=c2[23:12];
 assign G_ready=state==COLLECT;
 wire record_read=state==READ_REQ,record_write=state==WRITE||state==CLEAR;
 wire [7:0] record_address=state==CLEAR?{record_index,4'b0000}:{slot,word_base+{1'b0,word_index}};
 wire [35:0] record_data=state==CLEAR?36'd0:cache_read_data;
 motion_object_ram #(.WIDTH(36),.WORDS(256),.AB(8),.TWO_PORTS(0)) descriptors(
  .I_clk(I_clk),.A_enable(record_read||record_write),.A_write(record_write),.A_address(record_address),.A_data(record_data),.A_q(record_q),
  .B_enable(1'b0),.B_write(1'b0),.B_address(8'd0),.B_data(36'd0),.B_q());
 wire prop_read=state==ASSOC_REQ0||state==ASSOC_REQ1||state==BIRTH_START||state==BIRTH_Q0||state==PROP_MIN_FETCH||state==BIRTH_RANK_FETCH;
 wire prop_write=state==PROP_WRITE0||state==PROP_WRITE1;
 wire prop_word=state==ASSOC_REQ1||state==BIRTH_Q0||state==PROP_WRITE1||state==PROP_MIN_FETCH||state==BIRTH_RANK_FETCH;
 // Area ranking reuses the proposal statistics ERAM port in its own states.
 // FETCH launches the one-clock read; the following state consumes the value.
 // Proposal writes and ranking never overlap; no extra RAM or capacity loss.
 wire rank_read=state==PROP_MIN_FETCH||state==BIRTH_RANK_FETCH;
 wire [5:0] prop_address=rank_read?{min_index[4:0],1'b1}:{((state==PROP_WRITE0||state==PROP_WRITE1)?selected_prop[4:0]:prop_index[4:0]),prop_word};
 motion_object_ram #(.WIDTH(36),.WORDS(64),.AB(6),.TWO_PORTS(0)) proposals(
  .I_clk(I_clk),.A_enable(prop_read||prop_write),.A_write(prop_write),.A_address(prop_address),.A_data(prop_word?proposed_stats:proposed_box),.A_q(prop_q),
  .B_enable(1'b0),.B_write(1'b0),.B_address(6'd0),.B_data(36'd0),.B_q());
 function [14:0] mag;input signed [14:0] v;begin mag=v<0?-v:v;end endfunction
 function signed [11:0] minimum;input signed [11:0] a,b;begin minimum=a<b?a:b;end endfunction
 function signed [11:0] maximum;input signed [11:0] a,b;begin maximum=a>b?a:b;end endfunction
 function signed [7:0] bounded;input signed [13:0] v;begin bounded=v>32?32:v< -32?-32:v;end endfunction
 function [$clog2(WIDTH)-1:0] low_x;input signed [11:0] v;begin low_x=v<0?0:v;end endfunction
 function [$clog2(HEIGHT)-1:0] low_y;input signed [11:0] v;begin low_y=v<0?0:v;end endfunction
 function [$clog2(WIDTH)-1:0] high_x;input signed [11:0] v;begin high_x=v>WIDTH?WIDTH-1:v<=0?0:v-1'b1;end endfunction
 function [$clog2(HEIGHT)-1:0] high_y;input signed [11:0] v;begin high_y=v>HEIGHT?HEIGHT-1:v<=0?0:v-1'b1;end endfunction
 wire signed [11:0] rx0=c1[11:0],rx1=c1[23:12],ry0=c2[11:0],ry1=c2[23:12];
 motion_vector_at #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.LIVE_VECTORS(1)) interpolate(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_begin(vector_begin),.I_abort(I_abort),.I_x(I_camera_x),.I_y(I_camera_y),
  .I_x0(rx0),.I_x1(rx1),.I_y0(ry0),.I_y1(ry1),.O_done(vector_done),.O_x(vector_x),.O_y(vector_y));
 function signed [11:0] half_pair;input signed [11:0] a,b;reg signed [12:0] sum;
 begin sum=$signed(a)+$signed(b);half_pair=(sum>>>1)+(sum[12]&&sum[0]?1'b1:1'b0);end endfunction
 wire signed [11:0] centre_x=half_pair(rx0,rx1),centre_y=half_pair(ry0,ry1);
 wire signed [11:0] prop_centre_x=($signed({1'b0,prop_box[7:0]})+$signed({1'b0,prop_box[15:8]}))*2;
 wire signed [11:0] prop_centre_y=($signed({1'b0,prop_box[23:16]})+$signed({1'b0,prop_box[31:24]}))*2;
 wire signed [11:0] best_centre_x=($signed({1'b0,best_box[7:0]})+$signed({1'b0,best_box[15:8]}))*2;
 wire signed [11:0] best_centre_y=($signed({1'b0,best_box[23:16]})+$signed({1'b0,best_box[31:24]}))*2;
 wire signed [7:0] velocity_x=c3[7:0],velocity_y=c3[15:8];
 wire [14:0] association_distance=mag(centre_x+$signed(velocity_x)-prop_centre_x)+
   mag(centre_y+$signed(velocity_y)-prop_centre_y);
 wire [15:0] candidate_box_area=(best_box[15:8]-best_box[7:0])*(best_box[31:24]-best_box[23:16]);
 // Reject a new near-full-span, border-connected background proposal.
 // This birth-only gate does not delete or resize an established object.
 // The region worker already rejects exact full-frame components; include
 // their almost-full-frame variants without adding another area multiplier.
 wire background_birth=ROBUST_MATCH!=0&&candidate_box_area>=(WIDTH/4)*(HEIGHT/4)/3&&
  (((best_box[15:8]-best_box[7:0])>=3*(WIDTH/4)/4&&
     (best_box[7:0]<=2||best_box[15:8]>=WIDTH/4-2))||
   ((best_box[31:24]-best_box[23:16])>=3*(HEIGHT/4)/4&&
     (best_box[23:16]<=2||best_box[31:24]>=HEIGHT/4-2)));
 wire [19:0] old_box_area=((rx1-rx0)>>2)*((ry1-ry0)>>2);
 wire [21:0] candidate_area10=({6'b0,candidate_box_area}<<3)+({6'b0,candidate_box_area}<<1);
 wire [21:0] old_area3=({2'b0,old_box_area}<<1)+{2'b0,old_box_area};
 wire plausible=best_distance<48&&candidate_area10>old_area3&&candidate_box_area<old_area3;
 wire signed [11:0] residual_x=$signed(P_dx)*16-cam_x,residual_y=$signed(P_dy)*16-cam_y;
 wire signed [11:0] previous_x=c4[11:0],previous_y=c4[23:12],older_x=c5[11:0],older_y=c5[23:12];
 wire signed [14:0] residual_sum_x=$signed(residual_x)+$signed(previous_x)+(c0[19:18]>=2?$signed(older_x):15'sd0);
 wire signed [14:0] residual_sum_y=$signed(residual_y)+$signed(previous_y)+(c0[19:18]>=2?$signed(older_y):15'sd0);
 // Native mode accepts one analysis pixel per update, after camera
 // subtraction and the existing temporal/independent-evidence gates.
 // Previously a steady one-pixel motion never reached the two-pixel mean
 // threshold, regardless of how many frames it remained visible.
 wire [7:0] motion_floor=c0[19:18]>=2?(ROBUST_MATCH!=0?8'd48:8'd96):(ROBUST_MATCH!=0?8'd32:8'd64);
 wire moved=c0[19:18]!=0&&(mag(residual_sum_x)>=motion_floor||mag(residual_sum_y)>=motion_floor);
 // Require at least eight score units of improvement over camera flow.
 // The inclusive boundary admits measured12-vs20 weak/slow evidence, while
 // rejecting the27/29-vs34 background's five-unit initial confirmation.
 // The former strict greater-than-eight gate rejected quantised true motion.
 wire independent_evidence=ROBUST_MATCH==0||c0[34]||({1'b0,P_score}+17'd8<={1'b0,P_reference_score});
 wire qualified=moved&&c0[31:30]>=1&&independent_evidence;
 wire [1:0] next_hold=qualified?2:c0[33:32]!=0?c0[33:32]-1'b1:0;
 wire next_active=(c0[34]&&assist)||qualified||next_hold!=0;
 wire [11:0] candidate_x0={best_box[7:0],2'b00},candidate_x1={best_box[15:8],2'b00};
 wire [11:0] candidate_y0={best_box[23:16],2'b00},candidate_y1={best_box[31:24],2'b00};
 wire group_same_owner=c2[25:18]!=0&&c2[25:18]==c6[25:18];
 wire signed [12:0] overlap_x=$signed(minimum(c0[23:12],c4[23:12]))-$signed(maximum(c0[11:0],c4[11:0]));
 wire signed [12:0] overlap_y=$signed(minimum(c1[23:12],c5[23:12]))-$signed(maximum(c1[11:0],c5[11:0]));
 wire signed [12:0] smaller_width=minimum($signed(c0[23:12])-$signed(c0[11:0]),$signed(c4[23:12])-$signed(c4[11:0]));
 wire signed [12:0] smaller_height=minimum($signed(c1[23:12])-$signed(c1[11:0]),$signed(c5[23:12])-$signed(c5[11:0]));
 wire overlap_duplicate=overlap_x>0&&overlap_y>0&&(overlap_x<<<1)>=smaller_width&&(overlap_y<<<1)>=smaller_height;
 wire coherent_parts=c2[34]&&c6[34]&&((overlap_x>0&&(overlap_x<<<1)>=smaller_width)||(overlap_y>0&&(overlap_y<<<1)>=smaller_height));
 wire merge_owner_ok=c2[25:18]==0||c6[25:18]==0||group_same_owner||overlap_duplicate||coherent_parts;
 wire merge_velocity_ok=group_same_owner||(mag($signed(c2[17:10])-$signed(c6[17:10]))<=2&&mag($signed(c2[9:2])-$signed(c6[9:2]))<=2);
 wire merge_geometry_ok=maximum(c0[11:0],c4[11:0])<=minimum(c0[23:12],c4[23:12])+4&&
  maximum(c1[11:0],c5[11:0])<=minimum(c1[23:12],c5[23:12])+4;
 wire group_inside=$signed(group_x[23:12])>0&&$signed(group_y[23:12])>0&&$signed(group_x[11:0])<WIDTH&&$signed(group_y[11:0])<HEIGHT;
 // A native consumer needs only one mode-selected coordinate snapshot.
 // Keep the dual-bundle API as the default for legacy/test consumers.
 reg [TARGETS*16-1:0] legacy_mx0,legacy_mx1,legacy_my0,legacy_my1;
 assign O_mx0=SINGLE_OUTPUT!=0?O_x0:legacy_mx0;
 assign O_mx1=SINGLE_OUTPUT!=0?O_x1:legacy_mx1;
 assign O_my0=SINGLE_OUTPUT!=0?O_y0:legacy_my0;
 assign O_my1=SINGLE_OUTPUT!=0?O_y1:legacy_my1;
 wire [CB-1:0] selected_index=(SINGLE_OUTPUT!=0&&!assist)?moving_count:count;
 wire selected_group=SINGLE_OUTPUT==0||assist||group_meta[1];
 integer z;
 reg [35:0] typed_record_q;
 // Reserved payload bits are never state. Mask them at RAM load so generic
 // indexing cannot turn uninitialised/unused RAM bits into live FF/mux banks.
 always @*begin
  typed_record_q=record_q;
  if(word_base==0)case(word_index)
   1,2,4,5:typed_record_q={12'd0,record_q[23:0]};
   3:typed_record_q={20'd0,record_q[15:0]};
   6:typed_record_q={35'd0,record_q[0]};
   7:typed_record_q=0;
  endcase
  else if(word_base==8)case(word_index)
   0,1:typed_record_q={12'd0,record_q[23:0]};
   3:typed_record_q={20'd0,record_q[15:0]};
  endcase
 end
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
  state<=CLEAR;return_state<=0;assist<=0;regions_done<=0;limited<=0;slot<=0;record_index<=0;dup_index<=0;birth_slot<=0;merge_i<=0;merge_j<=0;group_index<=0;member_index<=0;
  word_index<=0;word_last<=0;word_base<=0;cache_high<=0;valid_mask<=0;cluster_mask<=0;next_uid<=1;prop_count<=0;prop_index<=0;min_index<=0;selected_prop<=0;prop_used<=0;minimum_area<=0;
  proposed_box<=0;proposed_stats<=0;prop_box<=0;best_distance<=0;best_prop<=0;best_box<=0;cam_x<=0;cam_y<=0;duplicate<=0;merge_changed<=0;uid_conflict<=0;
  group_x<=0;group_y<=0;group_meta<=0;group_members<=0;count<=0;moving_count<=0;
  O_busy<=1;O_done<=0;O_limited<=0;O_frame<=0;O_count<=0;O_moving_count<=0;O_x0<=0;O_x1<=0;O_y0<=0;O_y1<=0;legacy_mx0<=0;legacy_mx1<=0;legacy_my0<=0;legacy_my1<=0;O_bx0<=0;O_bx1<=0;O_by0<=0;O_by1<=0;O_observed<=0;
  P_begin<=0;P_learn<=0;P_cx<=0;P_cy<=0;P_tie_x<=0;P_tie_y<=0;vector_begin<=0;birth_seen<=0;
 end else begin
  O_done<=0;P_begin<=0;vector_begin<=0;
  if(G_done)regions_done<=1;if(G_overflow&&O_busy)limited<=1;
  if(I_abort)begin state<=CLEAR;record_index<=0;valid_mask<=0;cluster_mask<=0;O_busy<=1;regions_done<=0;end
  else case(state)
   CLEAR:if(record_index==(RECORDS-1))begin state<=IDLE;O_busy<=0;end else record_index<=record_index+1'b1;
   IDLE:if(I_begin)begin O_busy<=1;assist<=I_assist;O_frame<=I_frame;prop_count<=0;prop_used<=0;regions_done<=0;limited<=0;cluster_mask<=0;state<=COLLECT;end
   COLLECT:if(G_valid)begin
    proposed_box<={4'd0,G_y1,G_y0,G_x1,G_x0};proposed_stats<={21'd0,G_area};
    if(prop_count<32)begin selected_prop<=prop_count;prop_count<=prop_count+1'b1;state<=PROP_WRITE0;end
    else begin min_index<=0;minimum_area<=15'h7fff;selected_prop<=0;state<=PROP_MIN_FETCH;limited<=1;end
   end else if(regions_done)begin record_index<=0;state<=REC_START;end
   PROP_MIN_FETCH:state<=PROP_MIN;
   BIRTH_RANK_FETCH:state<=BIRTH_RANK;
   PROP_MIN:begin
    if(min_index<32&&prop_q[14:0]<minimum_area)begin minimum_area<=prop_q[14:0];selected_prop<=min_index;end
    if(min_index==32)begin
     if(proposed_stats[14:0]>minimum_area)begin state<=PROP_WRITE0;end else state<=COLLECT;
    end else begin min_index<=min_index+1'b1;state<=PROP_MIN_FETCH;end
   end
   PROP_WRITE0:state<=PROP_WRITE1;
   PROP_WRITE1:state<=COLLECT;
   READ_REQ:state<=READ_Q;
   READ_Q:begin
    case(cache_address)
     0:c0<=typed_record_q;1:c1<=typed_record_q[23:0];2:c2<=typed_record_q;3:c3<=typed_record_q[15:0];
     4:c4<=typed_record_q;5:c5<=typed_record_q[23:0];6:c6<=typed_record_q;7:c7<=typed_record_q[15:0];
    endcase
    if(word_index==word_last)state<=return_state;else begin word_index<=word_index+1'b1;state<=READ_REQ;end
   end
   WRITE:if(word_index==word_last)state<=return_state;else word_index<=word_index+1'b1;
   REC_START:begin slot<=record_index;word_base<=0;cache_high<=0;word_index<=0;word_last<=7;return_state<=REC_LOADED;state<=READ_REQ;end
   REC_LOADED:if(!c0[35])begin valid_mask[record_index]<=0;state<=REC_NEXT;end
   else begin
    vector_begin<=1;best_distance<=15'h7fff;best_prop<=63;best_box<=0;prop_index<=0;state<=VECTOR_WAIT;
   end
   VECTOR_WAIT:if(vector_done)begin
    cam_x<=vector_x;cam_y<=vector_y;
    if(prop_count==0)state<=SEARCH_START;else state<=ASSOC_REQ0;
   end
   ASSOC_REQ0:state<=ASSOC_Q0;
   ASSOC_Q0:begin prop_box<=prop_q;state<=ASSOC_REQ1;end
   ASSOC_REQ1:state<=ASSOC_Q1;
   ASSOC_Q1:begin
    if(!prop_used[prop_index]&&association_distance<best_distance)begin best_distance<=association_distance;best_prop<=prop_index;best_box<=prop_box[31:0];end
    state<=ASSOC_NEXT;
   end
   ASSOC_NEXT:if(prop_index+1==prop_count)state<=SEARCH_START;else begin prop_index<=prop_index+1'b1;state<=ASSOC_REQ0;end
   SEARCH_START:begin
    P_cx<=plausible?bounded(best_centre_x-centre_x):velocity_x;
    P_cy<=plausible?bounded(best_centre_y-centre_y):velocity_y;
    P_tie_x<=c0[34]&&assist||c0[33:32]!=0?velocity_x:bounded(cam_x/16);
    P_tie_y<=c0[34]&&assist||c0[33:32]!=0?velocity_y:bounded(cam_y/16);
    P_begin<=1;P_learn<=0;state<=SEARCH_WAIT;
   end
   SEARCH_WAIT:if(P_done)state<=SEARCH_APPLY;
   SEARCH_APPLY:begin
    if(P_quality&&P_score<(ROBUST_MATCH!=0?128:80))begin
     if(best_distance<48&&best_prop<32)prop_used[best_prop]<=1;
     c1[11:0]<=rx0+$signed(P_dx);c1[23:12]<=rx1+$signed(P_dx);
     c2[11:0]<=ry0+$signed(P_dy);c2[23:12]<=ry1+$signed(P_dy);
     c3[7:0]<=P_dx;c3[15:8]<=P_dy;c5<=c4;c4<={12'd0,residual_y,residual_x};
     c0[19:18]<=c0[19:18]==3?3:c0[19:18]+1'b1;
     c0[31:30]<=moved&&independent_evidence?(c0[31:30]==3?3:c0[31:30]+1'b1):0;
     c0[33:32]<=next_hold;c0[34]<=next_active;c0[29:26]<=0;
     c0[25:22]<=qualified?0:c0[25:22]+1'b1;c0[21:20]<=c0[21:20]+1'b1;
     c6[0]<=qualified;
     if(!next_active&&!qualified&&c0[25:22]>=7)begin c0[35]<=0;valid_mask[record_index]<=0;end
     if(qualified&&c0[21:20]==3&&best_distance<48)state<=REFRESH_START;else state<=REC_SAVE;
    end else begin
     c0[29:26]<=c0[29:26]==15?15:c0[29:26]+1'b1;
     if(c0[29:26]<2)begin c1[11:0]<=rx0+velocity_x;c1[23:12]<=rx1+velocity_x;c2[11:0]<=ry0+velocity_y;c2[23:12]<=ry1+velocity_y;end
     else c3[15:0]<=0;
     if(!assist)begin c0[33:32]<=c0[33:32]==0?0:c0[33:32]-1'b1;c0[34]<=c0[33:32]>1&&c0[29:26]<1;end
     if(c0[29:26]>=7)begin c0[35]<=0;c0[34]<=0;valid_mask[record_index]<=0;end
     c6[0]<=0;state<=REC_SAVE;
    end
   end
   REFRESH_START:begin P_begin<=1;P_learn<=1;state<=REFRESH_WAIT;end
   REFRESH_WAIT:if(P_done)state<=REC_SAVE;
   REC_SAVE:begin slot<=record_index;word_base<=0;cache_high<=0;word_index<=0;word_last<=7;return_state<=REC_CLUSTER;state<=WRITE;end
   REC_CLUSTER:if(c0[35]&&c0[34]&&(assist||c0[33:32]!=0))begin
    c2<={1'b1,c6[0],c0[17:10],c0[9:2],c3[7:0],c3[15:8],(c0[33:32]!=0),1'b0};
    c0<=c1;c1<=c2;c3<={20'd0,(16'b1<<record_index)};
    cluster_mask[record_index]<=1;word_base<=8;cache_high<=0;word_index<=0;word_last<=3;return_state<=REC_NEXT;state<=WRITE;
   end else state<=REC_NEXT;
   REC_NEXT:if(record_index==(RECORDS-1))begin birth_seen<=0;state<=BIRTH_SELECT;end else begin record_index<=record_index+1'b1;state<=REC_START;end
   BIRTH_SELECT:if(valid_mask=={RECORDS{1'b1}})state<=MERGE_START;
   else begin min_index<=0;minimum_area<=0;selected_prop<=63;state<=BIRTH_RANK_FETCH;end
   BIRTH_RANK:if(min_index==32)begin
    if(selected_prop==63)state<=MERGE_START;
    else begin prop_index<=selected_prop;birth_seen[selected_prop]<=1;state<=BIRTH_START;end
   end else begin
    if(min_index<prop_count&&!birth_seen[min_index]&&!prop_used[min_index]&&prop_q[14:0]>minimum_area)begin minimum_area<=prop_q[14:0];selected_prop<=min_index;end
    min_index<=min_index+1'b1;state<=BIRTH_RANK_FETCH;
   end
   BIRTH_START:if(prop_index>=prop_count)state<=MERGE_START;else if(prop_used[prop_index]||valid_mask=={RECORDS{1'b1}})state<=BIRTH_NEXT;else state<=BIRTH_Q0;
   BIRTH_Q0:begin prop_box<=prop_q;state<=BIRTH_Q1;end
   BIRTH_Q1:begin best_box<=prop_box[31:0];duplicate<=0;uid_conflict<=0;dup_index<=0;state<=DUP_START;end
   DUP_START:if(background_birth)state<=BIRTH_NEXT;
   else if(valid_mask[dup_index])begin
    slot<=dup_index;word_base<=0;cache_high<=0;word_index<=0;word_last<=2;return_state<=DUP_CHECK;state<=READ_REQ;
   end else state<=DUP_NEXT;
   DUP_CHECK:begin
    if($signed(candidate_x0)+4>=rx0&&$signed(candidate_x1)<=rx1+4&&
       $signed(candidate_y0)+4>=ry0&&$signed(candidate_y1)<=ry1+4)duplicate<=1;
    if(c0[17:10]==next_uid||c0[9:2]==next_uid)uid_conflict<=1;
    if(mag(centre_x-best_centre_x)+
      mag(centre_y-best_centre_y)<12)duplicate<=1;
    state<=DUP_NEXT;
   end
   DUP_NEXT:if(duplicate)state<=BIRTH_NEXT;
   else if(dup_index==(RECORDS-1)&&uid_conflict)begin
    next_uid<=next_uid==255?1:next_uid+1'b1;uid_conflict<=0;dup_index<=0;state<=DUP_START;
   end else if(dup_index==(RECORDS-1))begin
    for(z=RECORDS-1;z>=0;z=z-1)if(!valid_mask[z])begin slot<=z;birth_slot<=z;end
    c1<={12'd0,candidate_x1,candidate_x0};c2<={12'd0,candidate_y1,candidate_y0};state<=BIRTH_LEARN;
   end else begin dup_index<=dup_index+1'b1;state<=DUP_START;end
   BIRTH_LEARN:begin P_begin<=1;P_learn<=1;state<=BIRTH_WAIT;end
   BIRTH_WAIT:if(P_done)state<=BIRTH_APPLY;
   BIRTH_APPLY:if(P_quality)begin
    c0<={1'b1,1'b0,2'd0,2'd0,4'd0,4'd0,2'd0,2'd0,next_uid,8'd0,2'd0};
    c3<=0;c4<=0;c5<=0;c6<=0;c7<=0;valid_mask[birth_slot]<=1;next_uid<=next_uid==255?1:next_uid+1'b1;
    word_base<=0;word_index<=0;word_last<=7;cache_high<=0;return_state<=BIRTH_NEXT;state<=WRITE;
   end else state<=BIRTH_NEXT;
   BIRTH_NEXT:state<=BIRTH_SELECT;
   MERGE_START:begin merge_i<=0;merge_j<=0;merge_changed<=0;state<=MERGE_A;end
   MERGE_A:if(!cluster_mask[merge_i]||merge_i==0)state<=MERGE_NEXT;
   else begin slot<=merge_i;word_base<=8;word_index<=0;word_last<=3;cache_high<=0;return_state<=MERGE_A_DONE;state<=READ_REQ;end
   MERGE_A_DONE:begin merge_j<=0;state<=MERGE_B;end
   MERGE_B:if(merge_j>=merge_i)state<=MERGE_NEXT;
   else if(!cluster_mask[merge_j])merge_j<=merge_j+1'b1;
   else begin slot<=merge_j;word_base<=8;word_index<=0;word_last<=3;cache_high<=1;return_state<=MERGE_TEST;state<=READ_REQ;end
   MERGE_TEST:if(merge_owner_ok&&merge_velocity_ok&&merge_geometry_ok)begin
    c4[11:0]<=minimum(c0[11:0],c4[11:0]);c4[23:12]<=maximum(c0[23:12],c4[23:12]);
    c5[11:0]<=minimum(c1[11:0],c5[11:0]);c5[23:12]<=maximum(c1[23:12],c5[23:12]);
    c6[33:26]<=c2[33:26]<c6[33:26]?c2[33:26]:c6[33:26];
    c6[25:18]<=c6[25:18]!=0?c6[25:18]:c2[25:18];
    c6[34]<=c6[34]||c2[34];c6[1]<=c6[1]||c2[1];c7<=c7|c3;
    c2[35]<=0;cluster_mask[merge_i]<=0;merge_changed<=1;
    slot<=merge_j;word_base<=8;word_index<=0;word_last<=3;cache_high<=1;return_state<=MERGE_SAVE_A;state<=WRITE;
   end else begin merge_j<=merge_j+1'b1;state<=MERGE_B;end
   MERGE_SAVE_A:begin slot<=merge_i;word_base<=8;word_index<=0;word_last<=3;cache_high<=0;return_state<=MERGE_NEXT;state<=WRITE;end
   MERGE_NEXT:if(merge_i==(RECORDS-1))begin if(merge_changed)state<=MERGE_START;else state<=EMIT_START;end else begin merge_i<=merge_i+1'b1;state<=MERGE_A;end
   EMIT_START:begin group_index<=0;count<=0;moving_count<=0;O_observed<=0;O_bx0<=WIDTH-1;O_by0<=HEIGHT-1;O_bx1<=0;O_by1<=0;state<=EMIT_GROUP;end
   EMIT_GROUP:if(cluster_mask[group_index])begin slot<=group_index;word_base<=8;word_index<=0;word_last<=3;cache_high<=0;return_state<=EMIT_GROUP_DONE;state<=READ_REQ;end
   else state<=EMIT_NEXT;
   EMIT_GROUP_DONE:begin group_x<=c0;group_y<=c1;group_meta<=c2;group_members<=c3[15:0];member_index<=0;state<=OWNER_NEXT;end
   OWNER_NEXT:if(group_members[member_index])begin slot<=member_index;word_base<=0;word_index<=0;word_last<=0;cache_high<=1;return_state<=OWNER_META;state<=READ_REQ;end
   else state<=OWNER_WRITE;
   OWNER_META:begin c4[9:2]<=group_meta[25:18]!=0?group_meta[25:18]:group_meta[33:26];
    word_base<=0;word_index<=0;word_last<=0;cache_high<=1;return_state<=OWNER_WRITE;state<=WRITE;end
   OWNER_WRITE:if(member_index!=(RECORDS-1))begin member_index<=member_index+1'b1;state<=OWNER_NEXT;end
   else begin
    if(group_inside)begin
     if(selected_group&&selected_index<TARGETS)begin
      O_x0[selected_index*16+:16]<=low_x(group_x[11:0]);O_x1[selected_index*16+:16]<=high_x(group_x[23:12]);
      O_y0[selected_index*16+:16]<=low_y(group_y[11:0]);O_y1[selected_index*16+:16]<=high_y(group_y[23:12]);O_observed[selected_index]<=group_meta[34];
     end
     if(count<TARGETS)count<=count+1'b1;else limited<=1;
     if(group_meta[1])begin
      if(moving_count<TARGETS)begin
       if(SINGLE_OUTPUT==0)begin legacy_mx0[moving_count*16+:16]<=low_x(group_x[11:0]);legacy_mx1[moving_count*16+:16]<=high_x(group_x[23:12]);
        legacy_my0[moving_count*16+:16]<=low_y(group_y[11:0]);legacy_my1[moving_count*16+:16]<=high_y(group_y[23:12]);end
       if(low_x(group_x[11:0])<O_bx0)O_bx0<=low_x(group_x[11:0]);if(high_x(group_x[23:12])>O_bx1)O_bx1<=high_x(group_x[23:12]);
       if(low_y(group_y[11:0])<O_by0)O_by0<=low_y(group_y[11:0]);if(high_y(group_y[23:12])>O_by1)O_by1<=high_y(group_y[23:12]);
       moving_count<=moving_count+1'b1;end else limited<=1;
     end
    end
    state<=EMIT_NEXT;
   end
   EMIT_NEXT:if(group_index==(RECORDS-1))state<=FINISH;else begin group_index<=group_index+1'b1;state<=EMIT_GROUP;end
   FINISH:begin O_count<=count;O_moving_count<=moving_count;O_limited<=limited;O_busy<=0;O_done<=1;state<=IDLE;end
  endcase
 end
endmodule
