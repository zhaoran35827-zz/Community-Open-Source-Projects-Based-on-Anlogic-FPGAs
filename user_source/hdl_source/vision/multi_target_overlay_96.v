`timescale 1ns/1ps
// Competition build: keep the compact target-count panel and remove the
// multi-line diagnostic OSD from the synthesized pixel path.  The legacy
// diagnostic ports stay in the interface so an old project file can still
// instantiate this module, but they intentionally have no logic fanout.
module multi_target_overlay_96 #(parameter TARGETS=4, parameter CB=(TARGETS<8?3:4), parameter IB=(TARGETS<=2?1:$clog2(TARGETS)),parameter FIXED_HUD=0,UI_STYLE=0,MOTION_STYLE=0) (
    input I_valid,
    input [95:0] I_data,
    input [15:0] I_pixel_x,I_pixel_y,
    input [CB-1:0] I_count,
    input I_limit,
    input [4:0] I_reason,
    input [7:0] I_row_peak,
    input [5:0] I_labels,I_objects,
    input [15:0] I_dropped,
    input [4:0] I_frame_reason,
    input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
    output [95:0] O_data,input I_split,input I_motion_only
);
    function [7:0] label_char;
        input [3:0] column;
        input [CB-1:0] count;
        input limited;
        begin
            label_char=" ";
            case(column)
                0:label_char="R";1:label_char="E";2:label_char="D";3:label_char=":";
                5:label_char="0"+count;
                7:if(limited) label_char="L";
                8:if(limited) label_char="I";
                9:if(limited) label_char="M";
                10:if(limited) label_char="I";
                11:if(limited) label_char="T";
            endcase
        end
    endfunction
    generate if(TARGETS>4)begin:g_compact
        target_overlay_compact #(.MOTION_STYLE(MOTION_STYLE),.UI_STYLE(UI_STYLE),.TARGETS(TARGETS),.CB(CB),.FIXED_HUD(FIXED_HUD)) u_draw(
            .I_valid(I_valid),.I_data(I_data),.I_x(I_pixel_x),.I_y(I_pixel_y),
            .I_motion_only(I_motion_only),.I_count(I_count),.I_limit(I_limit),.I_xmin(I_xmin),.I_xmax(I_xmax),
            .I_ymin(I_ymin),.I_ymax(I_ymax),.O_data(O_data),.I_split(I_split));
    end else begin:g_legacy
    // Geometry is shared by all four pixels.  Valid boxes are bounded by
    // 1280x720, so centers and the +/-8 cross ranges fit 12/11 bits.
    wire [TARGETS-1:0] box_ok;
    wire [TARGETS*11-1:0] xlo,xhi;
    wire [TARGETS*10-1:0] ylo,yhi;
    wire [TARGETS*12-1:0] center_x,center_x_plus8;
    wire [TARGETS*11-1:0] center_y,center_y_plus8;
    wire [TARGETS-1:0] y_inside,y_edge,y_near1,y_near8;
    genvar obj;
    for(obj=0;obj<TARGETS;obj=obj+1) begin:g_geometry
        wire [15:0] xa=I_xmin[obj*16+:16],xb=I_xmax[obj*16+:16];
        wire [15:0] ya=I_ymin[obj*16+:16],yb=I_ymax[obj*16+:16];
        assign box_ok[obj]=obj<I_count && xa<=xb && ya<=yb && xb<1280 && yb<720 &&
            {1'b0,xa}+17'd3<={1'b0,xb} && {1'b0,ya}+17'd3<={1'b0,yb};
        assign xlo[obj*11+:11]=xa[10:0];
        assign xhi[obj*11+:11]=xb[10:0];
        assign ylo[obj*10+:10]=ya[9:0];
        assign yhi[obj*10+:10]=yb[9:0];
        wire [11:0] xs={1'b0,xa[10:0]}+{1'b0,xb[10:0]};
        wire [10:0] ys={1'b0,ya[9:0]}+{1'b0,yb[9:0]};
        wire [11:0] cx={1'b0,xs[11:1]};
        wire [10:0] cy={1'b0,ys[10:1]};
        assign center_x[obj*12+:12]=cx;
        assign center_x_plus8[obj*12+:12]=cx+12'd8;
        assign center_y[obj*11+:11]=cy;
        assign center_y_plus8[obj*11+:11]=cy+11'd8;
        assign y_inside[obj]=I_pixel_y>=ya && I_pixel_y<=yb;
        assign y_edge[obj]=I_pixel_y==ya || {1'b0,I_pixel_y}=={1'b0,ya}+17'd1 ||
            I_pixel_y==yb || {1'b0,I_pixel_y}+17'd1=={1'b0,yb};
        assign y_near1[obj]=I_pixel_y==cy || {1'b0,I_pixel_y}+17'd1==cy ||
            {1'b0,I_pixel_y}==cy+11'd1;
        assign y_near8[obj]=I_pixel_y<=cy+11'd8 && {1'b0,I_pixel_y}+17'd8>=cy;
    end
    genvar lane;
    for(lane=0;lane<4;lane=lane+1) begin:g_lane
        wire [15:0] px=I_pixel_x+lane;
        wire [15:0] lx=px-375;
        wire panel=px>=375 && px<583 && I_pixel_y>=208 && I_pixel_y<224;
        wire warning_on=I_limit;
        wire [15:0] bits;
        osd_char_lib font(.I_char(label_char(lx[7:4],I_count,warning_on)),
            .I_row(I_pixel_y[3:0]),.O_row_bits(bits));
        reg border,cross_on;
        wire [16:0] px_wide={1'b0,px};
        integer t;
        always @* begin
            border=0;cross_on=0;
            for(t=0;t<TARGETS;t=t+1) if(box_ok[t]) begin
                if(px>=xlo[t*11+:11] && px<=xhi[t*11+:11] && y_inside[t])
                    if(y_edge[t] || px==xlo[t*11+:11] || px_wide=={6'b0,xlo[t*11+:11]}+17'd1 ||
                        px==xhi[t*11+:11] || px_wide+17'd1=={6'b0,xhi[t*11+:11]}) border=1;
                if((y_near1[t] && px_wide+17'd8>=center_x[t*12+:12] && px<=center_x_plus8[t*12+:12]) ||
                   (y_near8[t] && (px==center_x[t*12+:12] || px_wide+17'd1==center_x[t*12+:12] ||
                        px_wide=={5'b0,center_x[t*12+:12]}+17'd1))) cross_on=1;
            end
        end
        assign O_data[(3-lane)*24 +:24]=!I_valid?I_data[(3-lane)*24 +:24]:
            panel?(bits[15-lx[3:0]]?
                (warning_on?24'hff4000:(UI_STYLE!=0?24'hdce8f2:24'hfff200)):(UI_STYLE!=0?24'h101b29:24'h202020)):
            cross_on?(UI_STYLE!=0?24'h61afef:24'hff00ff):border?(UI_STYLE!=0?24'h56d6b5:24'h00ff00):I_data[(3-lane)*24 +:24];
    end
    end endgenerate
endmodule

`timescale 1ns/1ps
// Sidecar: never changes the accepted CCL/mask, nor backpressures video.
// Sample one pixel per 4x4 cell in the previous-frame boxes. Rotation-
// invariant central polar moment distinguishes SOLID regular card shapes.
// UNKNOWN for small, contaminated, changing, or insufficiently compact ROIs.
module shape_moments_96 #(parameter TARGETS=4, parameter CB=(TARGETS<8?3:4), parameter IB=(TARGETS<=2?1:$clog2(TARGETS)), parameter ACCUM_ERAM=0,COORD_SCALE_SHIFT=0) (
    input I_clk,I_rst_n,I_valid,I_start,
    input [3:0] I_mask,
    input [15:0] I_x,I_y,
    input [CB-1:0] I_count,
    input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
    input [4:0] I_reason,
    output reg [TARGETS*2-1:0] O_shape,
    output reg [TARGETS*16-1:0] O_bcd_x,O_bcd_y,
    output reg O_text_ready
);
    function [15:0] bcd;
        input [11:0] v;
        reg [15:0] d;
        integer i,j;
        begin
            d=0;
            for(i=11;i>=0;i=i-1) begin
                for(j=0;j<4;j=j+1) if(d[j*4+:4]>=5) d[j*4+:4]=d[j*4+:4]+3;
                d={d[14:0],v[i]};
            end
            bcd=d;
        end
    endfunction
    function close_coord;
        input [15:0] a,b;
        begin close_coord=(a>=b)?a-b<=8:b-a<=8;end
    endfunction
    reg start_d;
    wire sof=I_start && !start_d;
    reg [TARGETS*16-1:0] rx0,rx1,ry0,ry1;
    reg [CB-1:0] rc;
    reg [TARGETS-1:0] bad;
    reg [15:0] n[0:TARGETS-1];
    reg [24:0] sx[0:TARGETS-1];
    reg [23:0] sy[0:TARGETS-1];
    reg [33:0] sr[0:TARGETS-1];
    reg busy,done;
    reg [IB-1:0] slot;
    wire [98:0] ram_stats;
    wire [15:0] n_slot=ACCUM_ERAM!=0?ram_stats[98:83]:n[slot];
    wire [24:0] sx_slot=ACCUM_ERAM!=0?ram_stats[82:58]:sx[slot];
    wire [23:0] sy_slot=ACCUM_ERAM!=0?ram_stats[57:34]:sy[slot];
    wire [33:0] sr_slot=ACCUM_ERAM!=0?ram_stats[33:0]:sr[slot];
    reg [TARGETS-1:0] hits;
    reg [IB-1:0] owner;
    integer t;
    always @* begin
        hits=0;owner=0;
        for(t=0;t<TARGETS;t=t+1)
            if(t<rc && I_x>=rx0[t*16+:16] && I_x<=rx1[t*16+:16] &&
                I_y>=ry0[t*16+:16] && I_y<=ry1[t*16+:16]) begin hits[t]=1;owner=t;end
    end
    // Pipeline coordinates/owner so multiplier logic is off the accumulator.
    reg sample;
    reg [IB-1:0] sample_owner;
    reg [8:0] sample_x;
    reg [7:0] sample_y;
    wire [17:0] xx=sample_x*sample_x;
    wire [15:0] yy=sample_y*sample_y;
    reg sample2;
    reg [IB-1:0] owner2;
    reg [8:0] x2;
    reg [7:0] y2;
    reg [18:0] r2;
    generate if(ACCUM_ERAM!=0)begin:g_accum_ram
        shape_accum_ram #(.TARGETS(TARGETS),.IB(IB)) u_stats(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(sof),
            .I_sample(sample2),.I_owner(owner2),.I_x(x2),.I_y(y2),.I_r2(r2),
            .I_read_slot(slot),.I_classify(busy),.O_stats(ram_stats));
    end else begin:g_accum_regs
        assign ram_stats=99'd0;
    end endgenerate
    reg [3:0] wait_end;
    reg [4:0] step;
    reg [TARGETS*2-1:0] result;
    reg [1:0] classification;
    // One registered, shared multiplier for all four final classifications.
    // 1280x720 at 4x4 sampling: N<=57600, sums fit these proven bounds.
    reg [31:0] ma;
    reg [33:0] mb;
    reg [65:0] product;
    reg [49:0] moment;
    reg [47:0] denom;
    wire [58:0] scaled={moment,9'b0};
    wire [54:0] wide_d={7'b0,denom};
    reg [54:0] threshold;
    reg [54:0] threshold_inc;
    wire [54:0] threshold_next=threshold+threshold_inc;
    wire below_threshold=scaled<=threshold;
    wire above_threshold=scaled>=threshold;
    reg square_low,triangle_low;
    always @* begin
        case(step)
            12,16,19:threshold_inc=wide_d<<1;
            13,14:threshold_inc=wide_d;
            15,17:threshold_inc=wide_d<<2;
            18:threshold_inc=wide_d<<5;
            default:threshold_inc=0;
        endcase
    end
    reg circle_ok,square_ok,triangle_ok;
    reg bcd_busy;
    reg [15:0] bcd_pipe;
    reg [IB-1:0] bcd_pipe_slot;
    reg bcd_pipe_axis,bcd_pipe_valid;
    reg [IB+1:0] bcd_step;
    reg [TARGETS*16-1:0] work_bx,work_by;
    wire [IB-1:0] bcd_slot=bcd_step[IB:1];
    wire [16:0] center_sum=bcd_step[0]?
        {1'b0,ry0[bcd_slot*16+:16]}+{1'b0,ry1[bcd_slot*16+:16]}:
        {1'b0,rx0[bcd_slot*16+:16]}+{1'b0,rx1[bcd_slot*16+:16]};
    wire [11:0] display_center=(center_sum>>1)<<COORD_SCALE_SHIFT;
    wire [15:0] next_bcd=bcd(display_center);
    integer j;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            start_d<=0;rx0<=0;rx1<=0;ry0<=0;ry1<=0;rc<=0;bad<=0;
            sample<=0;sample2<=0;sample_owner<=0;owner2<=0;
            sample_x<=0;sample_y<=0;x2<=0;y2<=0;r2<=0;
            wait_end<=0;busy<=0;done<=0;slot<=0;step<=0;result<=0;classification<=0;
            ma<=0;mb<=0;product<=0;moment<=0;denom<=0;
            threshold<=0;square_low<=0;triangle_low<=0;
            circle_ok<=0;square_ok<=0;triangle_ok<=0;
            O_shape<=0;O_bcd_x<=0;O_bcd_y<=0;O_text_ready<=0;
            bcd_busy<=0;bcd_step<=0;work_bx<=0;work_by<=0;
            bcd_pipe<=0;bcd_pipe_slot<=0;bcd_pipe_axis<=0;bcd_pipe_valid<=0;
            if(ACCUM_ERAM==0) for(j=0;j<TARGETS;j=j+1) begin n[j]<=0;sx[j]<=0;sy[j]<=0;sr[j]<=0;end
        end else begin
            start_d<=I_start;
            product<=ma*mb;
            sample<=I_valid && I_mask[3] && I_x[1:0]==0 && I_y[1:0]==0 &&
                hits!=0 && (hits & (hits-1'b1))==0 && !sof;
            sample_owner<=owner;sample_x<=I_x[10:2];sample_y<=I_y[9:2];
            sample2<=sample;owner2<=sample_owner;x2<=sample_x;y2<=sample_y;
            r2<={1'b0,xx}+{3'b0,yy};
            if(I_valid && I_mask[3] && (hits & (hits-1'b1))!=0) bad<=bad|hits;
            if(sample2 && ACCUM_ERAM==0) begin
                n[owner2]<=n[owner2]+1'b1;
                sx[owner2]<=sx[owner2]+x2;sy[owner2]<=sy[owner2]+y2;
                sr[owner2]<=sr[owner2]+r2;
            end
            if(I_valid && I_x==1276 && I_y==719) wait_end<=8;
            else if(wait_end!=0) begin
                wait_end<=wait_end-1'b1;
                if(wait_end==1) begin busy<=1;done<=0;slot<=0;step<=ACCUM_ERAM!=0?31:0;result<=0;end
            end
            if(busy) begin
                // Two clocks between operand selection and product consume.
                case(step)
                    0:begin ma<=n_slot;mb<=sr_slot;end
                    2:begin moment<=product[49:0];ma<=sx_slot;mb<=sx_slot;end
                    4:begin moment<=moment-product[49:0];ma<=sy_slot;mb<=sy_slot;end
                    6:begin moment<=moment-product[49:0];ma<=n_slot;mb<=n_slot;end
                    8:begin ma<=product[31:0];mb<=n_slot;end
                    10:denom<=product[47:0];
                    // Walk 80,82,83,84,88,90,94,126,128 times N^3.
                    // All increments reuse one adder and two comparators.
                    11:threshold<=(wide_d<<6)+(wide_d<<4);
                    12,13,16,19:threshold<=threshold_next;
                    14:begin circle_ok<=below_threshold;threshold<=threshold_next;end
                    15:begin square_low<=above_threshold;threshold<=threshold_next;end
                    17:begin square_ok<=square_low && below_threshold;threshold<=threshold_next;end
                    18:begin triangle_low<=above_threshold;threshold<=threshold_next;end
                    20:triangle_ok<=triangle_low && below_threshold;
                    21:begin
                        if(TARGETS>4)begin
                            classification<=0;
                            if(slot<rc && !bad[slot] && n_slot>=128 &&
                                {1'b0,rx0[slot*16+:16]}+17'd63<={1'b0,rx1[slot*16+:16]} &&
                                {1'b0,ry0[slot*16+:16]}+17'd63<={1'b0,ry1[slot*16+:16]} && denom!=0)begin
                                if(circle_ok)classification<=1;
                                else if(square_ok)classification<=2;
                                else if(triangle_ok)classification<=3;
                            end
                        end else begin
                            result[slot*2+:2]<=0;
                            if(slot<rc && !bad[slot] && n_slot>=128 &&
                                rx1[slot*16+:16]-rx0[slot*16+:16]>=63 &&
                                ry1[slot*16+:16]-ry0[slot*16+:16]>=63 && denom!=0)begin
                                if(circle_ok)result[slot*2+:2]<=1;
                                else if(square_ok)result[slot*2+:2]<=2;
                                else if(triangle_ok)result[slot*2+:2]<=3;
                            end
                            if(slot==TARGETS-1)begin busy<=0;done<=1;end
                            else slot<=slot+1'b1;
                        end
                    end
                    22:if(TARGETS>4)begin
                        for(j=0;j<TARGETS;j=j+1)if(slot==j)result[j*2+:2]<=classification;
                        if(slot==TARGETS-1)begin busy<=0;done<=1;end
                        else slot<=slot+1'b1;
                    end
                endcase
                if(step==(TARGETS>4?22:21)) step<=ACCUM_ERAM!=0?31:0;else if(step==31)step<=0;else step<=step+1'b1;
            end
            // One BCD conversion reused for eight coordinates. Publish all
            // eight together after nine clocks; suppress text until ready.
            if(bcd_busy)begin
                if(TARGETS>4)begin
                    bcd_pipe_valid<=bcd_step<TARGETS*2;
                    bcd_pipe<=next_bcd;bcd_pipe_slot<=bcd_slot;bcd_pipe_axis<=bcd_step[0];
                    if(bcd_pipe_valid)for(j=0;j<TARGETS;j=j+1)if(bcd_pipe_slot==j)begin
                        if(bcd_pipe_axis)work_by[j*16+:16]<=bcd_pipe;
                        else work_bx[j*16+:16]<=bcd_pipe;
                    end
                    if(bcd_step==TARGETS*2+1)begin
                        O_bcd_x<=work_bx;O_bcd_y<=work_by;O_text_ready<=1;bcd_busy<=0;
                    end else bcd_step<=bcd_step+1'b1;
                end else begin
                    if(bcd_step==TARGETS*2)begin
                        O_bcd_x<=work_bx;O_bcd_y<=work_by;O_text_ready<=1;bcd_busy<=0;
                    end else begin
                        if(bcd_step[0])work_by[bcd_slot*16+:16]<=next_bcd;
                        else work_bx[bcd_slot*16+:16]<=next_bcd;
                        bcd_step<=bcd_step+1'b1;
                    end
                end
            end
            if(sof) begin
                // Bind result to measured geometry, not just the output slot.
                for(j=0;j<TARGETS;j=j+1) begin
                    O_shape[j*2+:2]<=0;
                    if(done && !busy && I_reason==0 && j<I_count && j<rc &&
                        close_coord(I_xmin[j*16+:16],rx0[j*16+:16]) &&
                        close_coord(I_xmax[j*16+:16],rx1[j*16+:16]) &&
                        close_coord(I_ymin[j*16+:16],ry0[j*16+:16]) &&
                        close_coord(I_ymax[j*16+:16],ry1[j*16+:16]))
                        O_shape[j*2+:2]<=result[j*2+:2];
                    if(ACCUM_ERAM==0)begin n[j]<=0;sx[j]<=0;sy[j]<=0;sr[j]<=0;end
                end
                rx0<=I_xmin;rx1<=I_xmax;ry0<=I_ymin;ry1<=I_ymax;rc<=I_count;
                bad<=0;busy<=0;done<=0;wait_end<=0;sample<=0;sample2<=0;
                bcd_busy<=1;bcd_step<=0;O_text_ready<=0;bcd_pipe_valid<=0;
            end
        end
    end
endmodule
`timescale 1ns/1ps
// Two 10-character lines: "SQR I01   ", "X1234Y0567".
// Labels clamp at screen boundaries; near the top they necessarily move inside.
module shape_labels_96 #(parameter TARGETS=4, parameter CB=(TARGETS<8?3:4), parameter IB=(TARGETS<=2?1:$clog2(TARGETS)),parameter FIXED_HUD=0,UI_STYLE=0,TRACK_TAGS=0) (
    input I_valid,
    input I_ready,
    input [95:0] I_data,
    input [15:0] I_x,I_y,
    input [CB-1:0] I_count,
    input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
    input [TARGETS*2-1:0] I_shape,
    input [TARGETS*16-1:0] I_bcd_x,I_bcd_y,
    input [TARGETS*8-1:0] I_ids,
    output [95:0] O_data,input I_split
);
    function [7:0] char_at;
        input [3:0] col;
        input second;
        input [1:0] shape;
        input [15:0] bx,by;
        input [7:0] id;
        begin
            char_at=" ";
            if(second) case(col)
                0:char_at="X";
                1:char_at="0"+bx[15:12];
                2:char_at="0"+bx[11:8];
                3:char_at="0"+bx[7:4];
                4:char_at="0"+bx[3:0];
                5:char_at="Y";
                6:char_at="0"+by[15:12];
                7:char_at="0"+by[11:8];
                8:char_at="0"+by[7:4];
                9:char_at="0"+by[3:0];
            endcase
            else case(col)
                0:if(!second) case(shape) 1:char_at="C";2:char_at="S";3:char_at="T";default:char_at="U";endcase
                1:if(!second) case(shape) 1:char_at="I";2:char_at="Q";3:char_at="R";default:char_at="N";endcase
                2:if(!second) case(shape) 1:char_at="R";2:char_at="R";3:char_at="I";default:char_at="K";endcase
                4:char_at="I";
                5:char_at="0"+id[7:4];
                6:char_at="0"+id[3:0];
            endcase
            // Bit7 is reserved for motion metadata in the native renderer.
            // TRK is a tracking label, not a fabricated persistent object ID.
            if(!second && TRACK_TAGS!=0 && id[7])case(col)
                0:char_at="T";1:char_at="R";2:char_at="K";
                4,5,6:char_at=" ";
            endcase
        end
    endfunction
    // Video x is a multiple of four. Align panel origins to 16 pixels:
    // all four pixels share one glyph lookup, rather than four font ROMs.
        wire [15:0] px=I_x;
        reg panel;
        reg [7:0] ch;
        reg [3:0] fr,fc;
        reg [15:0] ax,ay,lx,ly,x0,x1,y0,y1;
        integer t;
        generate if(FIXED_HUD==0)begin:g_follow_labels
        always @* begin
            panel=0;ch=" ";fr=0;fc=0;
            ax=0;ay=0;lx=0;ly=0;x0=0;x1=0;y0=0;y1=0;
            for(t=0;t<TARGETS;t=t+1) begin
                x0=I_xmin[t*16+:16];x1=I_xmax[t*16+:16];
                y0=I_ymin[t*16+:16];y1=I_ymax[t*16+:16];
                ax=(x0>1120)?1120:{x0[15:4],4'b0};ay=(y0>=34)?y0-34:0;
                if(!panel && t<I_count && x0<=x1 && y0<=y1 && x1<1280 && y1<720 &&
                    x1-x0>=3 && y1-y0>=3 && px>=ax && px<ax+160 && I_y>=ay && I_y<ay+32) begin
                    panel=1;lx=px-ax;ly=I_y-ay;fr=ly[3:0];fc=lx[3:0];
                    ch=char_at(lx[7:4],ly[4],I_shape[t*2+:2],I_bcd_x[t*16+:16],I_bcd_y[t*16+:16],I_ids[t*8+:8]);
                end
            end
        end
        end else begin:g_fixed_labels
            // One selected metadata record + shared font. Fixed table avoids
            // six parallel box-relative anchor/subtract/clip networks.
            wire [15:0] base_x=I_split?16'd704:16'd608;
            wire [15:0] base_y=I_split?16'd208:16'd120;
            wire [15:0] relative_y=I_y-base_y;
            wire [IB-1:0] slot=relative_y[5+:IB];
            always @*begin
                lx=px-base_x;ly=relative_y;fr=ly[3:0];fc=lx[3:0];
                panel=px>=base_x && px<base_x+160 && I_y>=base_y &&
                    I_y<base_y+TARGETS*32 && slot<I_count;
                ch=char_at(lx[7:4],ly[4],I_shape[slot*2+:2],I_bcd_x[slot*16+:16],I_bcd_y[slot*16+:16],I_ids[slot*8+:8]);
                ax=0;ay=0;x0=0;x1=0;y0=0;y1=0;
            end
        end endgenerate
        wire [15:0] bits;
        osd_char_lib font(.I_char(ch),.I_row(fr),.O_row_bits(bits));
    genvar l;
    generate for(l=0;l<4;l=l+1) begin:g_lane
        assign O_data[(3-l)*24+:24]=I_valid && I_ready && panel?
            (bits[15-fc-l]?(UI_STYLE!=0?24'hdce8f2:24'hfff200):(UI_STYLE!=0?24'h101b29:24'h202020)):I_data[(3-l)*24+:24];
    end endgenerate
endmodule
`timescale 1ns/1ps
// Four-object sidecar. Does not change boxes, mask or CCL backpressure.
// Greedy global nearest eligible pair, one-to-one; not appearance tracking.
module target_tracks_96 #(
    parameter MAX_DISTANCE=128,
    parameter MAX_MISSED=8,
    parameter TARGETS=4, CB=(TARGETS<8?3:4), IB=(TARGETS<=2?1:$clog2(TARGETS)),DB_META=0,LIVE_BOX_INPUTS=0,BOX_DB=0
) (
    input I_clk,I_rst_n,I_start,
    input [CB-1:0] I_count,
    input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
    input [4:0] I_reason,
    output reg [TARGETS*8-1:0] O_ids,
    output reg O_ready,
    input [TARGETS-1:0] I_observed,input [TARGETS*7-1:0] I_preferred_tags,input [TARGETS-1:0] I_forget,
    output [TARGETS-1:0] O_track_active,
    output [TARGETS*7-1:0] O_track_tags,
    output [TARGETS*16-1:0] O_track_xmin,O_track_xmax,O_track_ymin,O_track_ymax,
    output reg [TARGETS-1:0] O_track_observed,
    input I_clear
);
    localparam LOAD=1,PREP=2,SCAN=3,MATCH=4,ALLOC=5,IDCHECK=6,
               NEXTDET=7,BUILD=8,PUBLISH=9,STORE=10;
    reg start_d;
    wire sof=I_start&&!start_d;
    reg [3:0] state;
    reg [IB-1:0] index,round,pick,best_d,best_t;
    reg [2*IB-1:0] scan_index;
    reg [TARGETS-1:0] det_valid,det_used,track_used,active;
    reg [11:0] best_cost;
    reg found;
    reg [CB-1:0] count;
    reg [TARGETS*16-1:0] bx0,bx1,by0,by1;
    reg [10:0] dx[0:TARGETS-1],dw[0:TARGETS-1],tx[0:TARGETS-1],tw[0:TARGETS-1];
    reg [9:0] dy[0:TARGETS-1],dh[0:TARGETS-1],ty[0:TARGETS-1],th[0:TARGETS-1];
    reg [6:0] tid[0:TARGETS-1],did[0:TARGETS-1],next_id;
    reg [3:0] age[0:TARGETS-1];
    reg [IB-1:0] destination[0:TARGETS-1];
    // Build next database separately; interrupted frames cannot half-commit.
    reg [TARGETS*11-1:0] nx,nw;
    reg [TARGETS*10-1:0] ny,nh;
    reg [TARGETS*7-1:0] ni;
    reg [TARGETS*4-1:0] na;
    reg [TARGETS-1:0] nv;
    reg [TARGETS-1:0] measured,next_measured;reg [TARGETS*7-1:0] preferred_tags;
    assign O_track_active=active;
    genvar slot;
    generate for(slot=0;slot<TARGETS;slot=slot+1)begin:g_track_snapshot
      wire [11:0] rx={1'b0,tx[slot]}+{1'b0,(tw[slot]>>1)}+tw[slot][0];
      wire [10:0] ry={1'b0,ty[slot]}+{1'b0,(th[slot]>>1)}+th[slot][0];
      assign O_track_tags[slot*7+:7]=tid[slot];
      wire [10:0] bounded_xlo=tx[slot]-(tw[slot]>>1);wire [9:0] bounded_ylo=ty[slot]-(th[slot]>>1);
      assign O_track_xmin[slot*16+:16]=BOX_DB!=0?{5'd0,tx[slot]}:tx[slot]<(tw[slot]>>1)?16'd0:{5'd0,bounded_xlo};
      assign O_track_xmax[slot*16+:16]=BOX_DB!=0?{5'd0,tw[slot]}:rx>1279?16'd1279:{5'd0,rx[10:0]};
      assign O_track_ymin[slot*16+:16]=BOX_DB!=0?{6'd0,ty[slot]}:ty[slot]<(th[slot]>>1)?16'd0:{6'd0,bounded_ylo};
      assign O_track_ymax[slot*16+:16]=BOX_DB!=0?{6'd0,th[slot]}:ry>719?16'd719:{6'd0,ry[9:0]};
    end endgenerate
    reg [TARGETS*8-1:0] display_ids;
    reg [61:0] build_record;
    wire [3:0] next_track_age=age[index]+1'b1;
    wire [IB-1:0] sd=scan_index[2*IB-1:IB],st=scan_index[IB-1:0];
    // Store legal endpoints directly, so six public boxes need no parallel
    // centre/width reconstruction. Association converts only the selected pair.
    wire [11:0] dsx={1'b0,dx[sd]}+{1'b0,dw[sd]},tsx={1'b0,tx[st]}+{1'b0,tw[st]};
    wire [10:0] dsy={1'b0,dy[sd]}+{1'b0,dh[sd]},tsy={1'b0,ty[st]}+{1'b0,th[st]};
    wire [10:0] dcx=BOX_DB!=0?dsx[11:1]:dx[sd],tcx=BOX_DB!=0?tsx[11:1]:tx[st];
    wire [9:0] dcy=BOX_DB!=0?dsy[10:1]:dy[sd],tcy=BOX_DB!=0?tsy[10:1]:ty[st];
    wire [10:0] dwidth=BOX_DB!=0?dw[sd]-dx[sd]:dw[sd],twidth=BOX_DB!=0?tw[st]-tx[st]:tw[st];
    wire [9:0] dheight=BOX_DB!=0?dh[sd]-dy[sd]:dh[sd],theight=BOX_DB!=0?th[st]-ty[st]:th[st];
    wire [10:0] ax=dcx>=tcx?dcx-tcx:tcx-dcx;
    wire [9:0] ay=dcy>=tcy?dcy-tcy:tcy-dcy;
    wire [10:0] aw=dwidth>=twidth?dwidth-twidth:twidth-dwidth;
    wire [9:0] ah=dheight>=theight?dheight-theight:theight-dheight;
    wire [11:0] distance={1'b0,ax}+{2'b0,ay};
    wire hinted=DB_META!=0&&preferred_tags[sd*7+:7]!=0;
    reg reserved_for_st;integer hint_slot;
    always @*begin reserved_for_st=0;
     for(hint_slot=0;hint_slot<TARGETS;hint_slot=hint_slot+1)
      if(det_valid[hint_slot]&&preferred_tags[hint_slot*7+:7]!=0&&preferred_tags[hint_slot*7+:7]==tid[st])reserved_for_st=1;
    end
    wire eligible=sd<TARGETS&&st<TARGETS&&det_valid[sd]&&!det_used[sd]&&active[st]&&!track_used[st]&&
     (hinted?(preferred_tags[sd*7+:7]==tid[st]):(!reserved_for_st&&distance<=MAX_DISTANCE&&aw<=({1'b0,twidth}>>1)+12'd8&&ah<=({1'b0,theight}>>1)+11'd8));
    wire [15:0] lx0=LIVE_BOX_INPUTS!=0?I_xmin[index*16+:16]:bx0[index*16+:16],lx1=LIVE_BOX_INPUTS!=0?I_xmax[index*16+:16]:bx1[index*16+:16];
    wire [15:0] ly0=LIVE_BOX_INPUTS!=0?I_ymin[index*16+:16]:by0[index*16+:16],ly1=LIVE_BOX_INPUTS!=0?I_ymax[index*16+:16]:by1[index*16+:16];
    wire [16:0] center_x={1'b0,lx0}+{1'b0,lx1};
    wire [16:0] center_y={1'b0,ly0}+{1'b0,ly1};
    reg free_found,id_collision,mapped;
    reg [IB-1:0] free_slot,mapped_det;
    reg [3:0] oldest;
    integer j;
    always @* begin
        free_found=0;free_slot=0;oldest=0;
        // Prefer empty slots. When full, evict the oldest unmatched slot.
        for(j=0;j<TARGETS;j=j+1) if(!free_found&&!active[j]&&!track_used[j]) begin
            free_found=1;free_slot=j;
        end
        if(!free_found) for(j=0;j<TARGETS;j=j+1)
            if(!track_used[j]&&(!free_found||age[j]>oldest)) begin
                free_found=1;free_slot=j;oldest=age[j];
            end
        id_collision=0;
        for(j=0;j<TARGETS;j=j+1) begin
            if(active[j]&&tid[j]==next_id) id_collision=1;
            if(det_used[j]&&did[j]==next_id) id_collision=1;
        end
        mapped=0;mapped_det=0;
        for(j=0;j<TARGETS;j=j+1) if(det_used[j]&&destination[j]==index) begin
            mapped=1;mapped_det=j;
        end
    end
    function [7:0] decimal_id;
        input [6:0] v;
        reg [7:0] b;
        integer k;
        begin
            b=0;
            for(k=6;k>=0;k=k-1) begin
                if(b[3:0]>=5)b[3:0]=b[3:0]+3;
                if(b[7:4]>=5)b[7:4]=b[7:4]+3;
                b={b[6:0],v[k]};
            end
            decimal_id=b;
        end
    endfunction
    integer q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            preferred_tags<=0;measured<=0;next_measured<=0;O_track_observed<=0;start_d<=0;state<=0;index<=0;round<=0;pick<=0;scan_index<=0;
            best_d<=0;best_t<=0;best_cost<=4095;found<=0;
            det_valid<=0;det_used<=0;track_used<=0;active<=0;count<=0;
            bx0<=0;bx1<=0;by0<=0;by1<=0;next_id<=1;
            nx<=0;nw<=0;ny<=0;nh<=0;ni<=0;na<=0;nv<=0;
            display_ids<=0;O_ids<=0;O_ready<=DB_META!=0;build_record<=0;
            for(q=0;q<TARGETS;q=q+1) begin
                dx[q]<=0;dy[q]<=0;dw[q]<=0;dh[q]<=0;
                tx[q]<=0;ty[q]<=0;tw[q]<=0;th[q]<=0;
                tid[q]<=0;did[q]<=0;age[q]<=0;destination[q]<=0;
            end
        end else if(DB_META!=0&&I_clear)begin
            // Inactive payload words may remain; association only uses active
            // slots. Clear state synchronously, avoiding a gated async reset.
            start_d<=0;state<=0;index<=0;round<=0;pick<=0;scan_index<=0;
            det_valid<=0;det_used<=0;track_used<=0;active<=0;count<=0;next_id<=1;
            O_ids<=0;O_ready<=1;O_track_observed<=0;measured<=0;next_measured<=0;preferred_tags<=0;nv<=0;
        end else begin
            start_d<=I_start;
            if(!sof) case(state)
                LOAD:begin
                    dx[index]<=BOX_DB!=0?lx0[10:0]:center_x[11:1];dy[index]<=BOX_DB!=0?ly0[9:0]:center_y[10:1];
                    dw[index]<=BOX_DB!=0?lx1[10:0]:lx1-lx0;dh[index]<=BOX_DB!=0?ly1[9:0]:ly1-ly0;
                    det_valid[index]<=index<count&&lx0<=lx1&&ly0<=ly1&&
                        lx1<1280&&ly1<720&&lx1-lx0>=3&&ly1-ly0>=3;
                    if(index==TARGETS-1) begin state<=PREP;round<=0;end
                    else index<=index+1'b1;
                end
                PREP:begin scan_index<=0;found<=0;best_cost<=4095;state<=SCAN;end
                SCAN:begin
                    if(eligible&&distance<best_cost) begin
                        found<=1;best_cost<=distance;best_d<=sd;best_t<=st;
                    end
                    if(scan_index==(2**(2*IB))-1)state<=MATCH;else scan_index<=scan_index+1'b1;
                end
                MATCH:begin
                    if(found) begin
                        det_used[best_d]<=1;track_used[best_t]<=1;
                        did[best_d]<=tid[best_t];destination[best_d]<=best_t;
                    end
                    if(!found||round==TARGETS-1) begin index<=0;state<=ALLOC;end
                    else begin round<=round+1'b1;state<=PREP;end
                end
                ALLOC:begin
                    if(det_valid[index]&&!det_used[index]&&free_found) begin
                        pick<=free_slot;state<=IDCHECK;
                    end else state<=NEXTDET;
                end
                IDCHECK:begin
                    if(id_collision) next_id<=next_id==99?1:next_id+1'b1;
                    else begin
                        did[index]<=next_id;destination[index]<=pick;
                        det_used[index]<=1;track_used[pick]<=1;
                        next_id<=next_id==99?1:next_id+1'b1;state<=NEXTDET;
                    end
                end
                NEXTDET:begin
                    if(index==TARGETS-1) begin index<=0;state<=BUILD;end
                    else begin index<=index+1'b1;state<=ALLOC;end
                end
                BUILD:begin
                    for(q=0;q<TARGETS;q=q+1)if(index==q)next_measured[q]<=DB_META!=0&&mapped&&measured[mapped_det];
                    if(TARGETS>4)begin
                        build_record[7:0]<=det_valid[index]&&det_used[index]?decimal_id(did[index]):0;
                        if(mapped)build_record[61:8]<={dx[mapped_det],dy[mapped_det],dw[mapped_det],dh[mapped_det],did[mapped_det],4'd0,1'b1};
                        else build_record[61:8]<={tx[index],ty[index],tw[index],th[index],tid[index],next_track_age,
                            active[index]&&age[index]<MAX_MISSED-1};
                        state<=STORE;
                    end else begin
                    for(q=0;q<TARGETS;q=q+1)if(index==q)begin
                    display_ids[q*8+:8]<=det_valid[index]&&det_used[index]?decimal_id(did[index]):0;
                    if(mapped) begin
                        nx[q*11+:11]<=dx[mapped_det];ny[q*10+:10]<=dy[mapped_det];
                        nw[q*11+:11]<=dw[mapped_det];nh[q*10+:10]<=dh[mapped_det];
                        ni[q*7+:7]<=did[mapped_det];na[q*4+:4]<=0;nv[q]<=1;
                    end else begin
                        nx[q*11+:11]<=tx[index];ny[q*10+:10]<=ty[index];
                        nw[q*11+:11]<=tw[index];nh[q*10+:10]<=th[index];
                        ni[q*7+:7]<=tid[index];
                        na[q*4+:4]<=age[index]+1'b1;
                        nv[q]<=active[index]&&age[index]<MAX_MISSED-1;
                    end
                    end
                    if(index==TARGETS-1)state<=PUBLISH;else index<=index+1'b1;
                    end
                end
                STORE:begin
                    for(q=0;q<TARGETS;q=q+1)if(index==q)begin
                        {nx[q*11+:11],ny[q*10+:10],nw[q*11+:11],nh[q*10+:10],ni[q*7+:7],na[q*4+:4],nv[q]}<=build_record[61:8];
                        display_ids[q*8+:8]<=build_record[7:0];
                    end
                    if(index==TARGETS-1)state<=PUBLISH;else begin index<=index+1'b1;state<=BUILD;end
                end
                PUBLISH:begin
                    for(q=0;q<TARGETS;q=q+1) begin
                        tx[q]<=nx[q*11+:11];ty[q]<=ny[q*10+:10];
                        tw[q]<=nw[q*11+:11];th[q]<=nh[q*10+:10];
                        tid[q]<=ni[q*7+:7];age[q]<=na[q*4+:4];
                    end
                    active<=nv;O_track_observed<=next_measured;O_ids<=display_ids;O_ready<=1;state<=0;
                end
            endcase
            if(sof) begin
                // Supersedes an unfinished association without database writes.
                state<=LOAD;index<=0;count<=I_count;measured<=DB_META!=0?I_observed:0;preferred_tags<=DB_META!=0?I_preferred_tags:0;next_measured<=0;
                if(DB_META!=0)active<=active&~I_forget;
                if(LIVE_BOX_INPUTS==0)begin bx0<=I_xmin;bx1<=I_xmax;by0<=I_ymin;by1<=I_ymax;end
                det_valid<=0;det_used<=0;track_used<=0;display_ids<=0;O_ready<=0;
                for(q=0;q<TARGETS;q=q+1)did[q]<=0;
                if(I_reason!=0) begin state<=0;O_ids<=0;O_ready<=1;end
                else if(I_count==0) begin
                    state<=0;active<=0;O_track_observed<=0;next_id<=1;O_ids<=0;O_ready<=1;
                end
            end
        end
    end
endmodule

// Eight-target statistics use five ERAMs instead of a growing register/mux
// bank. One read-modify-write per sampled pixel. Forward the last write to
// resolve back-to-back same-owner samples and cross-port collision behavior.
// SOF invalidates entries logically; RAM contents never need a reset sweep.
module shape_accum_ram #(parameter TARGETS=8,IB=3)(
 input I_clk,I_rst_n,I_start,I_sample,I_classify,
 input [IB-1:0] I_owner,I_read_slot,
 input [8:0] I_x,input [7:0] I_y,input [18:0] I_r2,
 output [98:0] O_stats
);
 reg [TARGETS-1:0] initialized;
 reg pending;
 reg [IB-1:0] owner;
 reg [8:0] sx;
 reg [7:0] sy;
 reg [18:0] r2;
 reg forwarded;
 reg [IB-1:0] last_owner;
 reg [98:0] last_value;
 wire [119:0] memory;
 wire [98:0] base=!initialized[owner]?99'd0:
     forwarded&&last_owner==owner?last_value:memory[98:0];
 wire [15:0] nn=base[98:83]+16'd1;
 wire [24:0] nx=base[82:58]+sx;
 wire [23:0] ny=base[57:34]+sy;
 wire [33:0] nr=base[33:0]+r2;
 wire [98:0] updated={nn,nx,ny,nr};
 wire we=I_rst_n && pending && !I_start;
 wire [9:0] ra=I_classify?I_read_slot:I_owner;
 wire [9:0] wa=owner;
 assign O_stats=initialized[I_read_slot]?memory[98:0]:99'd0;
 genvar b;
 generate for(b=0;b<3;b=b+1)begin:g_bank
  wire [119:0] padded={21'd0,updated};
  PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(40),.DATA_WIDTH_B(40),
   .ADDR_WIDTH_A(9),.ADDR_WIDTH_B(9),.DATA_DEPTH_A(512),.DATA_DEPTH_B(512),
   .MODE("PDPW"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
   .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
   .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
   .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
   .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
   .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
   .CLKMODE("SYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) ram(
   .doa(),.dia(padded[b*40+:40]),.addra(wa[8:0]),.cea(we),.ocea(1'b0),.clka(I_clk),.wea(we),.rsta(1'b0),.bea(1'b0),
   .dob(memory[b*40+:40]),.dib(40'd0),.addrb(ra[8:0]),.ceb(1'b1),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
   .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 end endgenerate
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   initialized<=0;pending<=0;owner<=0;sx<=0;sy<=0;r2<=0;
   forwarded<=0;last_owner<=0;last_value<=0;
  end else begin
   pending<=I_sample && !I_start;owner<=I_owner;sx<=I_x;sy<=I_y;r2<=I_r2;
   if(we)begin initialized[owner]<=1;forwarded<=1;last_owner<=owner;last_value<=updated;end
   if(I_start)begin initialized<=0;pending<=0;forwarded<=0;end
  end
 end
endmodule

// Four-lane-aligned renderer. Compare beat words once, then clip with 2-bit
// lane masks. All targets share the count-panel glyph lookup. No latency.
module target_overlay_compact #(parameter TARGETS=6,CB=3,VALIDATED_INPUTS=0,FIXED_HUD=0,UI_STYLE=0,MOTION_STYLE=0,SHOW_MOTION_HUD=1,IMG_WIDTH=1280,IMG_HEIGHT=720)(
 input I_valid,I_limit,input [95:0] I_data,input [15:0] I_x,I_y,
 input [CB-1:0] I_count,input [TARGETS*16-1:0] I_xmin,I_xmax,I_ymin,I_ymax,
 output [95:0] O_data,input I_split,input I_motion_only
);
 function [3:0] span;
  input [8:0] word_x;input [10:0] lo,hi;
  reg [3:0] lower,upper;
  begin
   lower=word_x==lo[10:2]?(4'b1111<<lo[1:0]):4'b1111;
   upper=word_x==hi[10:2]?(4'b1111>>(3-hi[1:0])):4'b1111;
   span=(word_x>=lo[10:2] && word_x<=hi[10:2])?(lower&upper):4'b0000;
  end
 endfunction
 wire [3:0] borders[0:TARGETS-1],crosses[0:TARGETS-1];
 genvar t;
 generate for(t=0;t<TARGETS;t=t+1)begin:g_object
  wire [15:0] xa=I_xmin[t*16+:16],xb=I_xmax[t*16+:16];
  wire [15:0] ya=I_ymin[t*16+:16],yb=I_ymax[t*16+:16];
  // Native producer supplies ordered, clipped boxes with span>=3.
  // Standalone/default mode retains all defensive input checks.
  wire ok=t<I_count && (VALIDATED_INPUTS!=0 || (xa<=xb && ya<=yb && xb<IMG_WIDTH && yb<IMG_HEIGHT && {1'b0,xa}+17'd3<={1'b0,xb} && {1'b0,ya}+17'd3<={1'b0,yb}));
  wire [11:0] xs={1'b0,xa[10:0]}+{1'b0,xb[10:0]};
  wire [11:0] ys={1'b0,ya[10:0]}+{1'b0,yb[10:0]};
  wire [10:0] cx=xs[11:1];wire [10:0] cy=ys[11:1];
  wire [10:0] left8=cx<8?11'd0:cx-11'd8;
  wire [10:0] left1=cx==0?11'd0:cx-11'd1;
  wire horizontal=I_y==ya || I_y==ya+1'b1 || I_y==yb || I_y+1'b1==yb;
  assign borders[t]=ok && I_x<IMG_WIDTH && I_y>=ya && I_y<=yb ?
    (horizontal?span(I_x[10:2],xa[10:0],xb[10:0]):
     span(I_x[10:2],xa[10:0],xa[10:0]+11'd1)|span(I_x[10:2],xb[10:0]-11'd1,xb[10:0])):4'd0;
  wire near1=I_y==cy || {1'b0,I_y}+17'd1==cy || I_y=={1'b0,cy}+11'd1;
  wire near8=I_y<={1'b0,cy}+11'd8 && {1'b0,I_y}+17'd8>=cy;
  assign crosses[t]=ok && I_x<IMG_WIDTH ?
    ((near1?span(I_x[10:2],left8,cx+11'd8):4'd0)|
     (near8?span(I_x[10:2],left1,cx+11'd1):4'd0)):4'd0;
 end endgenerate
 reg [3:0] border,cross_on;integer i;
 always @*begin
  border=0;cross_on=0;
  for(i=0;i<TARGETS;i=i+1)begin border=border|borders[i];cross_on=cross_on|crosses[i];end
 end
 wire [15:0] hud_x=FIXED_HUD!=0?(I_split?16'd704:16'd608):16'd608;
 wire [15:0] hud_y=FIXED_HUD!=0?(I_split?16'd176:16'd96):16'd208;
 wire [15:0] lx=I_x-hud_x;
 // Cell12 was blank; trim it in the new theme so full-screen count card
 // ends at x800, immediately before (never underneath) traffic card.
 wire panel=(SHOW_MOTION_HUD!=0 || !I_motion_only) && I_x>=hud_x && I_x<hud_x+(UI_STYLE!=0?192:208) && I_y>=hud_y && I_y<hud_y+16;
 reg [7:0] ch;
 always @*begin
  ch=" ";
  case(lx[7:4])
   0:ch=(MOTION_STYLE!=0&&I_motion_only)?"M":"R";1:ch=(MOTION_STYLE!=0&&I_motion_only)?"O":"E";2:ch=(MOTION_STYLE!=0&&I_motion_only)?"V":"D";3:ch=":";5:ch="0"+I_count;
   7:if(I_limit)ch="L";8:if(I_limit)ch="I";9:if(I_limit)ch="M";
   10:if(I_limit)ch="I";11:if(I_limit)ch="T";
  endcase
 end
 wire [15:0] glyph;
 osd_char_lib font(.I_char(ch),.I_row(I_y[3:0]),.O_row_bits(glyph));
 genvar lane;
 generate for(lane=0;lane<4;lane=lane+1)begin:g_lane
  assign O_data[(3-lane)*24+:24]=!I_valid?I_data[(3-lane)*24+:24]:
   panel?(glyph[15-lx[3:0]-lane]?(I_limit?24'hff4000:(UI_STYLE!=0?24'hdce8f2:24'hfff200)):(UI_STYLE!=0?24'h101b29:24'h202020)):
   (cross_on[lane] && !(MOTION_STYLE!=0&&I_motion_only))?(UI_STYLE!=0?24'h61afef:24'hff00ff):border[lane]?(UI_STYLE!=0?24'h56d6b5:24'h00ff00):I_data[(3-lane)*24+:24];
 end endgenerate
endmodule
