`timescale 1ns/1ps
// Four regional translations,25 +/-8-analysis-pixel candidates;64-row anchors. Sparse RGB332
// patches and SAD scores use ERAM. ONE score adder is time-multiplexed.
// Start with previous-frame velocity; refresh from current-frame anchor rows.
// A majority of anchor points must support a nonzero candidate.
// Local translations approximate modest camera rotation; no blanket motion veto.
// Selected geometry must fit COLS*ROWS*25<=1024 and <=15 points/quadrant.
module camera_shift_fine_96 #(parameter IMG_WIDTH=640,IMG_HEIGHT=360)(
 input I_clk,I_rst_n,I_enable,I_valid,I_start,
 input [8:0] I_beat,input [10:0] I_y,input [7:0] I_signature,
 output reg [23:0] O_vectors,output [23:0] O_frame_vectors);
 localparam COLS=IMG_WIDTH/128,ROWS=(IMG_HEIGHT+23)/64;
 localparam LEFT_COLS=(IMG_WIDTH/2+63)/128;
 reg [2:0] top_rows_seen,bottom_rows_seen;
 reg image_bank,frame_seen,reference_valid;
 wire wb=I_start?!image_bank:image_bank;
 wire patch_x=I_beat[4:0]>=14 && I_beat[4:0]<=18;
 wire patch_y=I_y[5:0]>=24 && I_y[5:0]<=40 && I_y[1:0]==0;
 wire center=I_valid && I_enable && I_beat[4:0]==16 && I_y[5:0]==32 && (I_y>>6)<ROWS;
 wire [3:0] col=I_beat>>5;wire [3:0] row=I_y>>6;
 wire [2:0] patch_col=I_beat[4:0]-14;
 wire [2:0] patch_row=(I_y[5:0]-24)>>2;
 // Explicit constant shift/add keeps sparse-address arithmetic out of DSP.
 // Ten-bit modular result matches the original address expression.
 function [9:0] row_offset;input [3:0] value;integer k;reg [9:0] sum;
 begin sum=0;for(k=0;k<8;k=k+1)if((COLS>>k)&1)sum=sum+({6'd0,value}<<k);row_offset=sum;end endfunction
 wire [9:0] patch_tile=row_offset(row)+col;
 wire [9:0] patch_base=(patch_tile<<4)+(patch_tile<<3)+patch_tile;
 wire [9:0] patch_row_offset={7'd0,patch_row}+{5'd0,patch_row,2'b00};
 wire [9:0] patch_at=patch_base+patch_row_offset+patch_col;
 wire patch_write=I_valid && I_enable && patch_x && patch_y && row<ROWS && col<COLS;
 reg busy,last_burst;reg [4:0] phase;reg [9:0] base;
 wire [4:0] candidate=phase==0?12:(phase<=12?phase-1'b1:phase);
 reg [1:0] quadrant;reg [7:0] current;
 wire [15:0] patch_out;
 wire [7:0] old_patch=image_bank?patch_out[7:0]:patch_out[15:8];
 genvar f;
 generate for(f=0;f<2;f=f+1)begin:g_patch
 wire wr=patch_write && wb==f;wire rd=busy && image_bank!=f;
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(8),.DATA_WIDTH_B(8),
 .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
 .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
 .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),.OREGSET_A("RESET"),.OREGSET_B("RESET"),
 .SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
 .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory(
 .doa(patch_out[f*8+:8]),.dia(I_signature),.addra(wr?patch_at:base+candidate),
 .cea(wr || rd),.ocea(1'b0),.clka(I_clk),.wea(wr),.rsta(1'b0),.bea(1'b0),
 .dob(),.dib(8'd0),.addrb(10'd0),.ceb(1'b0),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 end endgenerate
 function [2:0] dist3;input [2:0] a,b;begin dist3=a>=b?a-b:b-a;end endfunction
 function [1:0] dist2;input [1:0] a,b;begin dist2=a>=b?a-b:b-a;end endfunction
 wire [4:0] difference={2'd0,dist3(current[7:5],old_patch[7:5])}+
     {2'd0,dist3(current[4:2],old_patch[4:2])}+{3'd0,dist2(current[1:0],old_patch[1:0])};
 reg ref_valid,score_valid;reg [4:0] ref_phase,score_phase;
 reg [1:0] ref_quad,score_quad;reg [4:0] diff_d,identity_difference;reg vote_d;
 reg [6:0] clear_left,clear_at;
 reg scan;reg [1:0] scan_q;reg [4:0] scan_i;
 reg scan_valid;reg [1:0] scan_q_d;reg [4:0] scan_i_d;
 wire [12:0] score_read;
 wire [6:0] score_address=scan?(({5'd0,scan_q}<<4)+({5'd0,scan_q}<<3)+scan_q+scan_i):(({5'd0,ref_quad}<<4)+({5'd0,ref_quad}<<3)+ref_quad+ref_phase);
 wire [6:0] score_write_address=clear_left!=0?clear_at:(({5'd0,score_quad}<<4)+({5'd0,score_quad}<<3)+score_quad+score_phase);
 wire [8:0] sum_update=score_read[8:0]+diff_d;
 wire [3:0] votes_update=score_read[12:9]+vote_d;
 wire [12:0] score_update={votes_update,sum_update};
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(13),.DATA_WIDTH_B(13),
 .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
 .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
 .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),.OREGSET_A("RESET"),.OREGSET_B("RESET"),
 .SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
 .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) scores(
 .doa(score_read),.dia(13'd0),.addra({3'd0,score_address}),
 .cea(ref_valid || scan),.ocea(1'b0),.clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),
 .dob(),.dib(clear_left!=0?13'd0:score_update),
 .addrb({3'd0,score_write_address}),
 .ceb(clear_left!=0 || score_valid),.oceb(1'b0),.clkb(I_clk),.web(clear_left!=0 || score_valid),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 function [5:0] vector;input [4:0] p;begin
 case(p)
 0:vector=6'b110110;
 1:vector=6'b110111;
 2:vector=6'b110000;
 3:vector=6'b110001;
 4:vector=6'b110010;
 5:vector=6'b111110;
 6:vector=6'b111111;
 7:vector=6'b111000;
 8:vector=6'b111001;
 9:vector=6'b111010;
 10:vector=6'b000110;
 11:vector=6'b000111;
 12:vector=6'b000000;
 13:vector=6'b000001;
 14:vector=6'b000010;
 15:vector=6'b001110;
 16:vector=6'b001111;
 17:vector=6'b001000;
 18:vector=6'b001001;
 19:vector=6'b001010;
 20:vector=6'b010110;
 21:vector=6'b010111;
 22:vector=6'b010000;
 23:vector=6'b010001;
 24:vector=6'b010010;
 default:vector=0;endcase end endfunction
 function [7:0] points;input [1:0] q;reg [7:0] cols;reg [2:0] seen;integer bit_index;
 begin cols=q[0]?COLS-LEFT_COLS:LEFT_COLS;seen=q[1]?bottom_rows_seen:top_rows_seen;
 points=0;for(bit_index=0;bit_index<3;bit_index=bit_index+1)if(seen[bit_index])points=points+(cols<<bit_index); end endfunction
 wire [7:0] seen_points=points(scan_q_d);
 wire [7:0] needed_votes=(seen_points>>1)+seen_points[0];
 wire [9:0] fit_limit={2'b0,seen_points}+{1'b0,seen_points,1'b0};
 reg [23:0] pending_vectors;reg [8:0] best_score,zero_score;reg [4:0] best_index;reg [3:0] best_votes;
 assign O_frame_vectors=I_start?(frame_seen?pending_vectors:24'd0):O_vectors;
 wire take=scan_i_d==0 || score_read[8:0]<best_score || (score_read[8:0]==best_score && scan_i_d==12);
 wire [8:0] final_score=take?score_read[8:0]:best_score;
 wire [3:0] final_votes=take?score_read[12:9]:best_votes;
 wire [4:0] final_index=take?scan_i_d:best_index;
 always @(posedge I_clk or negedge I_rst_n)
 if(!I_rst_n)begin
 image_bank<=0;frame_seen<=0;reference_valid<=0;O_vectors<=0;pending_vectors<=0;
 busy<=0;last_burst<=0;phase<=0;base<=0;quadrant<=0;current<=0;
 ref_valid<=0;score_valid<=0;ref_phase<=0;score_phase<=0;ref_quad<=0;score_quad<=0;diff_d<=0;
 clear_left<=0;clear_at<=0;scan<=0;scan_q<=0;scan_i<=0;scan_valid<=0;scan_q_d<=0;scan_i_d<=0;
 best_score<=0;zero_score<=0;best_index<=12;best_votes<=0;identity_difference<=0;vote_d<=0;top_rows_seen<=0;bottom_rows_seen<=0;
 end else if(!I_enable)begin
 frame_seen<=0;reference_valid<=0;O_vectors<=0;pending_vectors<=0;
 busy<=0;ref_valid<=0;score_valid<=0;scan<=0;scan_valid<=0;clear_left<=0;
 end else begin
 ref_valid<=busy;ref_phase<=candidate;ref_quad<=quadrant;
 score_valid<=ref_valid;score_phase<=ref_phase;score_quad<=ref_quad;diff_d<=difference>4?5'd4:difference;
 vote_d<=ref_phase!=12 && {1'b0,difference}+6'd1<{1'b0,identity_difference};
 if(ref_valid && ref_phase==12)identity_difference<=difference;
 if(center)begin busy<=1;phase<=0;base<=patch_base;current<=I_signature;
 quadrant<={I_y>=IMG_HEIGHT/2,({I_beat,2'b00}>=IMG_WIDTH/2)};
 last_burst<=col==COLS-1;
 if(col==COLS-1)begin if(I_y>=IMG_HEIGHT/2)bottom_rows_seen<=bottom_rows_seen+1'b1;
 else top_rows_seen<=top_rows_seen+1'b1;end end
 else if(busy)begin if(phase==24)busy<=0;else phase<=phase+1'b1;end
 if(clear_left!=0)begin clear_left<=clear_left-1'b1;clear_at<=clear_at+1'b1;end
 if(score_valid && score_phase==24 && last_burst)begin scan<=1;scan_q<=0;scan_i<=0;end
 scan_valid<=scan;scan_q_d<=scan_q;scan_i_d<=scan_i;
 if(scan)begin
 if(scan_i==24)begin scan_i<=0;if(scan_q==3)scan<=0;else scan_q<=scan_q+1'b1;end
 else scan_i<=scan_i+1'b1;end
 if(scan_valid)begin
 if(take)begin best_score<=score_read[8:0];best_index<=scan_i_d;best_votes<=score_read[12:9];end
 if(scan_i_d==12)zero_score<=score_read[8:0];
 if(scan_i_d==24)begin
 // An incomplete early-row scan is not evidence for a stationary camera.
 // Keep the last valid regional estimate until this frame has enough anchors.
 if(reference_valid && seen_points>=4)begin
 if(final_index!=12 &&
 final_votes>=needed_votes &&
 final_score<zero_score-(zero_score>>2) && final_score<=fit_limit)
 begin pending_vectors[scan_q_d*6+:6]<=vector(final_index);O_vectors[scan_q_d*6+:6]<=vector(final_index);end
 else begin pending_vectors[scan_q_d*6+:6]<=0;O_vectors[scan_q_d*6+:6]<=0;end
 end end end
 if(I_start)begin
 image_bank<=!image_bank;frame_seen<=1;reference_valid<=frame_seen;
 O_vectors<=frame_seen?pending_vectors:24'd0;pending_vectors<=frame_seen?pending_vectors:24'd0;
 clear_left<=100;clear_at<=0;busy<=0;ref_valid<=0;score_valid<=0;scan<=0;scan_valid<=0;top_rows_seen<=0;bottom_rows_seen<=0;
 end end
endmodule
