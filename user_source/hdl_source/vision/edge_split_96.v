`timescale 1ns/1ps
module edge_split_96 #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,EDGE_THRESHOLD=6,
 parameter COLOR_EDGES=0,SHARED_OUTLINE=0,COLOR_THRESHOLD=2,OUTLINE_THRESHOLD=6,NOISE_REJECT=0,
 parameter REFINED_EDGES=0,FIELD_FIX=0,VISUAL_PROFILE=0
)(input I_clk,I_rst_n,I_valid,I_start,I_last,I_enable,
 input [95:0] I_data,I_original,input I_cartoon_enable,input [3:0] I_graphics_mask,
 output O_valid,O_start,O_last,output [95:0] O_data,input [3:0] I_precomputed_edges);
 generate if(VISUAL_PROFILE==2)begin:g_shared_cache
 edge_visual_cache_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_view(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(I_valid),.I_start(I_start),.I_enable(I_enable),.I_data(I_data),.I_edges(I_precomputed_edges),
 .O_valid(O_valid),.O_start(O_start),.O_last(O_last),.O_data(O_data));
 end else if(VISUAL_PROFILE!=0)begin:g_visual
 edge_visual_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.OUTLINE_ENABLE(SHARED_OUTLINE)) u_view(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(I_valid),.I_start(I_start),.I_last(I_last),.I_enable(I_enable),
 .I_data(I_data),.I_original(I_original),.I_cartoon_enable(I_cartoon_enable),.I_graphics_mask(I_graphics_mask),
 .O_valid(O_valid),.O_start(O_start),.O_last(O_last),.O_data(O_data));
 end else begin:g_legacy
 edge_split_legacy_96 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.EDGE_THRESHOLD(EDGE_THRESHOLD),
 .COLOR_EDGES(COLOR_EDGES),.SHARED_OUTLINE(SHARED_OUTLINE),.COLOR_THRESHOLD(COLOR_THRESHOLD),
 .OUTLINE_THRESHOLD(OUTLINE_THRESHOLD),.NOISE_REJECT(NOISE_REJECT),.REFINED_EDGES(REFINED_EDGES),.FIELD_FIX(FIELD_FIX)) u_view(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(I_valid),.I_start(I_start),.I_last(I_last),.I_enable(I_enable),
 .I_data(I_data),.I_original(I_original),.I_cartoon_enable(I_cartoon_enable),.I_graphics_mask(I_graphics_mask),
 .O_valid(O_valid),.O_start(O_start),.O_last(O_last),.O_data(O_data));
 end endgenerate
endmodule
`timescale 1ns/1ps
// Center crop, NO scaling: source [W/4,3W/4) in both half screens.
// All modes use the same two ping-pong line banks. No DDR/clock/reset changes.
// One 20-bit grayscale history + five 20-bit dual-port image RAMs = 6 ERAMs.
// Causal Roberts 2x2 gradient, one-pixel/one-row neighborhood. First row/column
// are black. Four independent pixels/beat; no horizontal pair decimation.
module edge_split_legacy_96 #(
    parameter IMG_WIDTH=1280,IMG_HEIGHT=720,EDGE_THRESHOLD=6,
    parameter COLOR_EDGES=0,SHARED_OUTLINE=0,COLOR_THRESHOLD=2,OUTLINE_THRESHOLD=6,NOISE_REJECT=0,
    parameter REFINED_EDGES=0, FIELD_FIX=0
)(
    input I_clk,I_rst_n,I_valid,I_start,I_last,I_enable,
    input [95:0] I_data,I_original,
    input I_cartoon_enable,input [3:0] I_graphics_mask,
    output reg O_valid,O_start,O_last,output reg [95:0] O_data
);
    localparam BEATS=IMG_WIDTH/4,HALF=BEATS/2,CROP=BEATS/4;
    // Supported geometry: even height; width divisible by 16, <=2048 pixels.
    reg [8:0] x;reg [10:0] y;reg mode;
    wire [8:0] px=I_start?9'd0:x;
    wire [10:0] py=I_start?11'd0:y;
    wire active=I_start?I_enable:mode;
    // Left uses the fully composed color stream; right uses only I_original.
    wire [95:0] source=I_data;
    wire [19:0] gray,previous_gray;
    genvar n;
    generate for(n=0;n<4;n=n+1)begin:g_gray
        wire [23:0] p=I_original[(3-n)*24+:24];
        assign gray[(3-n)*5+:5]={2'd0,p[23:21]}+{1'd0,p[15:12]}+{2'd0,p[7:5]};
    end endgenerate
    generate if(COLOR_EDGES==0)begin:g_legacy_gray
    PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),
        .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
        .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("NORMAL"),
        .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
        .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
        .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
        .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
        .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) u_gray (
        .doa(previous_gray),.dia(gray),.addra({1'b0,px}),
        .cea(I_valid),.ocea(1'b0),.clka(I_clk),.wea(I_valid),.rsta(1'b0),.bea(1'b0),
        .dob(),.dib(20'd0),.addrb(10'd0),.ceb(1'b0),.oceb(1'b0),
        .clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
        .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
    end else begin:g_no_legacy_gray
        assign previous_gray=0;
    end endgenerate
    reg v1,m1,c1;reg [3:0] protect1;reg [8:0] x1;reg [10:0] y1;
    reg [19:0] gray1;reg [95:0] data1;
    reg [4:0] upper_left,lower_left;
    wire [3:0] edges,strong_edges;
    function [4:0] distance;
        input [4:0] a,b;begin distance=a>=b?a-b:b-a;end
    endfunction
    // Ratio-based red membership is less affected by a common illumination
    // scale than an absolute gradient. This assists contour only, never CCL.
    function red_family;input [14:0] p;reg [5:0] g,b;
    begin g={1'b0,p[9:5]}+{2'b0,p[9:6]}+6'd1;
          b={1'b0,p[4:0]}+{2'b0,p[4:1]}+6'd1;
          red_family=p[14:10]>=4 && {1'b0,p[14:10]}>=g && {1'b0,p[14:10]}>=b;
    end endfunction
    generate if(COLOR_EDGES==0)begin:g_legacy_edges
    assign strong_edges=0;
    for(n=0;n<4;n=n+1)begin:g_edge
        wire [4:0] a,c;
        if(n==0)begin:g_first
            assign a=upper_left;assign c=lower_left;
        end else begin:g_next
            assign a=previous_gray[(4-n)*5+:5];assign c=gray1[(4-n)*5+:5];
        end
        wire [4:0] b=previous_gray[(3-n)*5+:5];
        
        wire [4:0] d=gray1[(3-n)*5+:5];
        wire [5:0] magnitude={1'b0,distance(a,d)}+{1'b0,distance(b,c)};
        assign edges[3-n]=y1!=0 && (x1!=0 || n!=0) && magnitude>=EDGE_THRESHOLD;
    end
    end else begin:g_color_edges
        // One RGB555 history shared by cartoon outlines AND binary display.
        // 60-bit line = 3 ERAM, no full-frame color/label buffer. Distinct
        // colors with equal luminance remain detectable. Weak/strong decisions
        // reuse the same horizontal/vertical channel differences.
        wire [59:0] current,above;
        reg [59:0] current1;
        reg [14:0] left;
        reg [2:0] left_field_sign;reg left_field_pair,left_field_hard;
        wire [11:0] field_hsign,field_vsign;wire [3:0] field_hpair,field_vpair;
        wire [3:0] wh,wv,hh,hv,lh,lv;
        localparam SUPPORT_BITS=FIELD_FIX!=0?20:12;
        wire [SUPPORT_BITS-1:0] previous_support;
        wire [11:0] legacy_support={(y1!=0?previous_support[3:0]:4'd0),(REFINED_EDGES!=0?lh:(lh|hh)),wh};
        wire [19:0] field_support={field_hsign,hh,field_hpair};
        wire [SUPPORT_BITS-1:0] support_write=FIELD_FIX!=0?field_support:legacy_support;
        reg left_v_weak,left_v_weak2,left_v_link;
        if(NOISE_REJECT!=0)begin:g_support
            // Weak evidence and linked/hard evidence, one bit each per lane.
            // Read previous row; write current decisions one cycle later.
            PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(SUPPORT_BITS),.DATA_WIDTH_B(SUPPORT_BITS),
                .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
                .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
                .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
                .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
                .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
                .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
                .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
                .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory (
                .doa(previous_support),.dia({SUPPORT_BITS{1'b0}}),.addra({1'b0,px}),
                .cea(I_valid),.ocea(1'b0),.clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),
                .dob(),.dib(support_write),.addrb({1'b0,x1}),.ceb(v1),.oceb(1'b0),
                .clkb(I_clk),.web(v1),.rstb(1'b0),.beb(1'b0),
                .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
        end else assign previous_support=0;
        for(n=0;n<4;n=n+1)begin:g_pack
            wire [23:0] p=I_original[(3-n)*24+:24];
            assign current[(3-n)*15+:15]={p[23:19],p[15:11],p[7:3]};
        end
        for(n=0;n<3;n=n+1)begin:g_history
            PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),
                .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
                .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
                .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("NORMAL"),
                .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
                .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
                .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
                .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
                .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory (
                .doa(above[n*20+:20]),.dia(current[n*20+:20]),.addra({1'b0,px}),
                .cea(I_valid),.ocea(1'b0),.clka(I_clk),.wea(I_valid),.rsta(1'b0),.bea(1'b0),
                .dob(),.dib(20'd0),.addrb(10'd0),.ceb(1'b0),.oceb(1'b0),
                .clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
                .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
        end
        if(FIELD_FIX!=0)begin:g_field_boundaries
        // Three tangent samples must agree on qualified RGB polarity.
        // A >=24-level seed has finite reach; no recursive weak propagation,
        // red-membership bypass, averaged-gradient adders, or extra image RAM.
        for(n=0;n<4;n=n+1)begin:g_boundary
            wire [14:0] center=current1[(3-n)*15+:15],up=above[(3-n)*15+:15];
            wire [14:0] prev;
            if(n==0)assign prev=left;else assign prev=current1[(4-n)*15+:15];
            wire [2:0] weak_h,weak_v,hard_h,hard_v,strong_h,strong_v,hsgn,vsgn;
            genvar ch;
            for(ch=0;ch<3;ch=ch+1)begin:g_channel
                wire [4:0] dh=distance(center[ch*5+:5],prev[ch*5+:5]);
                wire [4:0] dv=distance(center[ch*5+:5],up[ch*5+:5]);
                assign weak_h[ch]=dh>=COLOR_THRESHOLD;assign weak_v[ch]=dv>=COLOR_THRESHOLD;
                assign hard_h[ch]=dh>=COLOR_THRESHOLD+1;assign hard_v[ch]=dv>=COLOR_THRESHOLD+1;
                assign strong_h[ch]=dh>=OUTLINE_THRESHOLD;assign strong_v[ch]=dv>=OUTLINE_THRESHOLD;
                assign hsgn[ch]=weak_h[ch] && center[ch*5+:5]>prev[ch*5+:5];
                assign vsgn[ch]=weak_v[ch] && center[ch*5+:5]>up[ch*5+:5];
            end
            assign wh[3-n]=(x1!=0 || n!=0) && |weak_h;
            assign wv[3-n]=y1!=0 && |weak_v;
            assign hh[3-n]=(x1!=0 || n!=0) && |hard_h;
            assign hv[3-n]=y1!=0 && |hard_v;
            assign field_hsign[(3-n)*3+:3]=hsgn;
            assign field_vsign[(3-n)*3+:3]=vsgn;
            wire hmatch=hsgn==previous_support[8+(3-n)*3+:3];
            assign field_hpair[3-n]=y1!=0 && wh[3-n] && previous_support[4+3-n] && hmatch;
            assign lh[3-n]=y1>=2 && wh[3-n] && previous_support[3-n] && hmatch;
            wire ph,pp;wire [2:0] ps;
            if(n==0)begin assign ph=x1!=0&&left_field_hard;assign pp=x1!=0&&left_field_pair;assign ps=left_field_sign;end
            else begin assign ph=hv[4-n];assign pp=field_vpair[4-n];assign ps=field_vsign[(4-n)*3+:3];end
            assign field_vpair[3-n]=wv[3-n] && ph && vsgn==ps;
            assign lv[3-n]=wv[3-n] && pp && vsgn==ps;
            assign edges[3-n]=lh[3-n] || lv[3-n];
            assign strong_edges[3-n]=(lh[3-n] && |strong_h) || (lv[3-n] && |strong_v);
        end
        end else begin:g_compatible_boundaries
        assign field_hsign=0;assign field_vsign=0;assign field_hpair=0;assign field_vpair=0;
        for(n=0;n<4;n=n+1)begin:g_boundary
            wire [14:0] center=current1[(3-n)*15+:15];
            wire [14:0] up=above[(3-n)*15+:15];
            wire [14:0] prev;
            if(n==0)assign prev=left;
            else assign prev=current1[(4-n)*15+:15];
            wire [2:0] weak_h,weak_v,strong_h,strong_v;
            wire [2:0] hard_h,hard_v;
            genvar ch;
            for(ch=0;ch<3;ch=ch+1)begin:g_channel
                wire [4:0] dh=distance(center[ch*5+:5],prev[ch*5+:5]);
                wire [4:0] dv=distance(center[ch*5+:5],up[ch*5+:5]);
                assign weak_h[ch]=dh>=(NOISE_REJECT!=0?1:COLOR_THRESHOLD);
                assign weak_v[ch]=dv>=(NOISE_REJECT!=0?1:COLOR_THRESHOLD);
                assign hard_h[ch]=dh>=COLOR_THRESHOLD;
                assign hard_v[ch]=dv>=COLOR_THRESHOLD;
                assign strong_h[ch]=dh>=OUTLINE_THRESHOLD;
                assign strong_v[ch]=dv>=OUTLINE_THRESHOLD;
            end
            // No false wrap between lines or stale previous-frame first row.
            wire red_h=REFINED_EDGES!=0 && (red_family(center)!=red_family(prev));
            wire red_v=REFINED_EDGES!=0 && (red_family(center)!=red_family(up));
            wire coherent_h=(weak_h[0]&&weak_h[1]) || (weak_h[0]&&weak_h[2]) || (weak_h[1]&&weak_h[2]);
            wire coherent_v=(weak_v[0]&&weak_v[1]) || (weak_v[0]&&weak_v[2]) || (weak_v[1]&&weak_v[2]);
            assign wh[3-n]=(x1!=0 || n!=0) && (REFINED_EDGES!=0?(|hard_h || coherent_h || red_h):|weak_h);
            assign wv[3-n]=y1!=0 && (REFINED_EDGES!=0?(|hard_v || coherent_v || red_v):|weak_v);
            assign hh[3-n]=(x1!=0 || n!=0) && (|hard_h || red_h);
            assign hv[3-n]=y1!=0 && (|hard_v || red_v);
            wire pw,pw2,pl;
            if(n==0)begin
                assign pw=x1!=0 && left_v_weak;
                assign pw2=x1!=0 && left_v_weak2;
                assign pl=x1!=0 && left_v_link;
            end else begin
                assign pw=wv[4-n];assign pl=REFINED_EDGES!=0?lv[4-n]:(hv[4-n] || lv[4-n]);
                if(n==1)assign pw2=x1!=0 && left_v_weak;
                else assign pw2=wv[5-n];
            end
            // Directional support removes a single bright/dark pixel's edges.
            // Once seeded by >=16 levels, connected >=8-level segments survive.
            assign lh[3-n]=y1!=0 && wh[3-n] &&
                (previous_support[7-n] || (hh[3-n] && previous_support[3-n] &&
                  (REFINED_EDGES==0 || red_h || previous_support[11-n])));
            assign lv[3-n]=wv[3-n] && (pl || (hv[3-n] && pw && (REFINED_EDGES==0 || red_v || pw2)));
            assign edges[3-n]=NOISE_REJECT!=0?(lh[3-n] || lv[3-n]):
                (((x1!=0 || n!=0) && |weak_h) || (y1!=0 && |weak_v));
            assign strong_edges[3-n]=NOISE_REJECT!=0?
                ((lh[3-n] && (|strong_h || red_h)) || (lv[3-n] && (|strong_v || red_v))):
                (((x1!=0 || n!=0) && |strong_h) || (y1!=0 && |strong_v));
        end
        end
        always @(posedge I_clk or negedge I_rst_n)
            if(!I_rst_n)begin current1<=0;left<=0;left_field_sign<=0;left_field_pair<=0;left_field_hard<=0;left_v_weak<=0;left_v_weak2<=0;left_v_link<=0;end
            else begin current1<=current;if(v1)begin left<=current1[14:0];left_field_sign<=field_vsign[2:0];left_field_pair<=field_vpair[0];left_field_hard<=hv[0];
                left_v_weak<=wv[0];left_v_weak2<=wv[1];left_v_link<=FIELD_FIX!=0?hv[0]:REFINED_EDGES!=0?lv[0]:(hv[0] || lv[0]);end end
    end endgenerate
    reg flushing,flush_bank,flush_mode;reg [8:0] flush_x;
    wire emit=flushing || (v1 && y1!=0);
    wire [8:0] out_x=flushing?flush_x:x1;
    wire out_mode=flushing?flush_mode:m1;
    wire out_bank=flushing?flush_bank:!y1[0];
    wire [8:0] offset=out_mode ? CROP+((out_x>=HALF)?out_x-HALF:out_x):out_x;
    wire [9:0] read_address={out_bank,offset};
    wire [9:0] write_address={y1[0],x1};
    wire [99:0] cache_read;
    wire [95:0] outlined;
    generate for(n=0;n<4;n=n+1)begin:g_outline
        assign outlined[n*24+:24]=SHARED_OUTLINE!=0 && c1 && strong_edges[n] && !protect1[n]
            ?24'd0:data1[n*24+:24];
    end endgenerate
    wire [99:0] cache_write={edges,outlined};
    generate for(n=0;n<5;n=n+1)begin:g_line
        PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),
            .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
            .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
            .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
            .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
            .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
            .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
            .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
            .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory (
            .doa(),.dia(cache_write[n*20+:20]),.addra(write_address),
            .cea(v1),.ocea(1'b0),.clka(I_clk),.wea(v1),.rsta(1'b0),.bea(1'b0),
            .dob(cache_read[n*20+:20]),.dib(20'd0),.addrb(read_address),
            .ceb(emit),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
            .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
    end endgenerate
    reg emit_d,start_d,last_d,binary_d;
    wire first_out=!flushing && y1==1 && x1==0;
    integer lane;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            x<=0;y<=0;mode<=0;c1<=0;protect1<=0;v1<=0;m1<=0;x1<=0;y1<=0;gray1<=0;data1<=0;
            upper_left<=0;lower_left<=0;flushing<=0;flush_x<=0;flush_bank<=0;flush_mode<=0;
            emit_d<=0;start_d<=0;last_d<=0;binary_d<=0;
            O_valid<=0;O_start<=0;O_last<=0;O_data<=0;
        end else begin
            if(I_start)begin mode<=I_enable;x<=0;y<=0;end
            if(I_valid)begin
                if(px==BEATS-1)begin x<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;end
                else begin x<=px+1'b1;y<=py;end
            end
            v1<=I_valid;m1<=active;c1<=I_cartoon_enable;protect1<=I_graphics_mask;x1<=px;y1<=py;gray1<=gray;data1<=source;
            if(v1)begin
                upper_left<=previous_gray[4:0];lower_left<=gray1[4:0];
                if(x1==BEATS-1 && y1==IMG_HEIGHT-1)begin
                    flushing<=1;flush_x<=0;flush_bank<=y1[0];flush_mode<=m1;
                end
            end
            if(flushing)begin
                if(flush_x==BEATS-1)begin flushing<=0;flush_x<=0;end
                else flush_x<=flush_x+1'b1;
            end
            emit_d<=emit;start_d<=emit&&first_out;last_d<=emit&&out_x==BEATS-1;
            binary_d<=out_mode&&out_x>=HALF;
            O_valid<=emit_d;O_start<=start_d;O_last<=last_d;O_data<=cache_read[95:0];
            if(binary_d)for(lane=0;lane<4;lane=lane+1)
                O_data[lane*24+:24]<={24{cache_read[96+lane]}};
        end
    end
endmodule

// Effect profile: centred 3x3 Sobel/Gaussian, 8-bit luminance. Original
// RGB/chroma and graphics travel through the SAME centre-pixel row/word.
// Five 20K row RAMs plus the five existing crop-cache RAMs. No frame RAM.
module edge_visual_96 #(parameter IMG_WIDTH=1280,IMG_HEIGHT=720,OUTLINE_ENABLE=1)(
 input I_clk,I_rst_n,I_valid,I_start,I_last,I_enable,
 input [95:0] I_data,I_original,input I_cartoon_enable,input [3:0] I_graphics_mask,
 output reg O_valid,O_start,O_last,output reg [95:0] O_data);
 localparam BEATS=IMG_WIDTH/4,HALF=BEATS/2,CROP=BEATS/4;
 reg [8:0] x;reg [10:0] y;reg mode;
 reg flush;reg [8:0] fx;
 wire run=I_valid || flush;
 wire [8:0] rx=flush?fx:I_start?9'd0:x;
 wire [10:0] ry=flush?IMG_HEIGHT:I_start?11'd0:y;
 wire [31:0] gray;
 genvar n;
 generate for(n=0;n<4;n=n+1)begin:g_gray
  wire [23:0] p=I_original[(3-n)*24+:24];
  wire [9:0] sum={2'd0,p[23:16]}+{1'd0,p[15:8],1'b0}+{2'd0,p[7:0]}+10'd2;
  assign gray[(3-n)*8+:8]=sum[9:2];
 end endgenerate
 wire [143:0] line_in={10'd0,(I_start?I_enable:mode),I_cartoon_enable,I_graphics_mask,I_data,gray};
 wire [143:0] line_out;
 generate for(n=0;n<4;n=n+1)begin:g_rgb_row
  visual_row_ram #(.WIDTH(36)) m(.clk(I_clk),.en(run),.we(I_valid&&!flush),.addr(rx),
   .di(line_in[n*36+:36]),.dout(line_out[n*36+:36]));
 end endgenerate
 reg v1,virtual1,v2;reg [8:0] x1,x2;reg [10:0] y1,y2;
 reg [31:0] bottom1,bottom2;reg [143:0] middle2;
 wire [31:0] top2;
 visual_row_ram #(.WIDTH(32)) top_row(.clk(I_clk),.en(v1),.we(v1&&!virtual1),.addr(x1),
  .di(line_out[31:0]),.dout(top2));
 wire kernel_valid=v2 && y2!=0;
 wire [10:0] cy=y2-1'b1;
 wire [31:0] mid=middle2[31:0],top=y2==1?middle2[31:0]:top2;
 reg [15:0] tl,ml,bl;
 function [7:0] at;
  input [31:0] word4;input [15:0] left2;input integer idx;input first;
  begin
   if(idx<0)at=first?word4[31:24]:(idx==-2?left2[15:8]:left2[7:0]);
   else if(idx>3)at=word4[7:0];else at=word4[31-idx*8-:8];
  end
 endfunction
 wire [10:0] mag[0:4];wire [7:0] smooth[0:4];wire [5:0] direction[0:4];
 generate for(n=0;n<5;n=n+1)begin:g_sobel
  wire [7:0] a=at(top,tl,n-2,x2==0),b=at(top,tl,n-1,x2==0),c=at(top,tl,n,x2==0);
  wire [7:0] d=at(mid,ml,n-2,x2==0),e=at(mid,ml,n-1,x2==0),f=at(mid,ml,n,x2==0);
  wire [7:0] g=at(bottom2,bl,n-2,x2==0),h=at(bottom2,bl,n-1,x2==0),i=at(bottom2,bl,n,x2==0);
  wire [9:0] right={2'd0,c}+{1'd0,f,1'b0}+{2'd0,i};
  wire [9:0] leftv={2'd0,a}+{1'd0,d,1'b0}+{2'd0,g};
  wire [9:0] down={2'd0,g}+{1'd0,h,1'b0}+{2'd0,i};
  wire [9:0] up={2'd0,a}+{1'd0,b,1'b0}+{2'd0,c};
  wire [9:0] dx=right>=leftv?right-leftv:leftv-right;
  wire [9:0] dy=down>=up?down-up:up-down;
  assign mag[n]={1'b0,dx}+{1'b0,dy};
  // Qualified scalar gradients; unlike the old RGB555 test these retain
  // sub-bin slopes and do not require three channel masks to match exactly.
  assign direction[n]={right>leftv,dx>=40,dx>=16,down>up,dy>=40,dy>=16};
  wire [11:0] gs={4'd0,a}+{3'd0,b,1'b0}+{4'd0,c}+{3'd0,d,1'b0}+{2'd0,e,2'b00}+{3'd0,f,1'b0}+{4'd0,g}+{3'd0,h,1'b0}+{4'd0,i}+12'd8;
  assign smooth[n]=gs[11:4];
 end endgenerate
 // The result at n=0 belongs to the PREVIOUS word's last pixel. Retain
 // n=1..3, then join n=0 of the next word; no one-pixel contour displacement.
 reg pending,pending_last;reg [8:0] pending_x;reg [10:0] pending_y;
 reg [101:0] pending_rgb;
 reg [32:0] pmag;reg [23:0] psmooth;
 reg [10:0] end_mag;reg [7:0] end_smooth;reg [17:0] pdir;reg [5:0] end_dir;
 wire send=pending && (pending_last || (kernel_valid && x2!=0));
 wire [43:0] mags={pmag,(pending_last?end_mag:mag[0])};
 wire [31:0] smoothed={psmooth,(pending_last?end_smooth:smooth[0])};
 wire [23:0] directions={pdir,(pending_last?end_dir:direction[0])};
 reg fv;reg [8:0] vx;reg [10:0] vy;reg [101:0] vrgb;
 reg [43:0] vmags;reg [31:0] vsmoothed;reg [23:0] vdir;
 wire [19:0] support_read,support_write;
 wire [3:0] edges,hpair,vpair;
 reg vleft_sign,vleft_weak,vleft_pair,vleft_hard;
 // One finite-support row, no recursive weak propagation or frame buffer.
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
 .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),.CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) support(
 .doa(support_read),.dia(20'd0),.addra({1'b0,pending_x}),.cea(send),.ocea(1'b0),.clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),
 .dob(),.dib(support_write),.addrb({1'b0,vx}),.ceb(fv),.oceb(1'b0),.clkb(I_clk),.web(fv),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 wire [95:0] colored;
 generate for(n=0;n<4;n=n+1)begin:g_color
  wire [5:0] code=vdir[(3-n)*6+:6];
  wire hs=code[5],hh=code[4],hw=code[3],vs=code[2],vh=code[1],vw=code[0];
  wire [3:0] above_code=support_read[(3-n)*4+:4];
  wire hmatch=hs==above_code[3];
  assign hpair[3-n]=vy!=0 && hw && above_code[1] && hmatch && (hh||above_code[2]);
  wire ps,pw,pp,ph;
  if(n==0)begin assign ps=vleft_sign;assign pw=vx!=0&&vleft_weak;assign pp=vx!=0&&vleft_pair;assign ph=vleft_hard;end
  else begin assign ps=vdir[(4-n)*6+2];assign pw=vdir[(4-n)*6];assign pp=vpair[4-n];assign ph=vdir[(4-n)*6+1];end
  assign vpair[3-n]=vw && pw && vs==ps && (vh||ph);
  assign edges[3-n]=(vy>=2 && hw && above_code[0] && hmatch) || (vw && pp && vs==ps);
  assign support_write[(3-n)*4+:4]={hs,hh,hw,hpair[3-n]};
  visual_color_pixel #(.OUTLINE_ENABLE(OUTLINE_ENABLE)) p(
   .rgb(vrgb[(3-n)*24+:24]),.smoothed(vsmoothed[(3-n)*8+:8]),
   .enable(vrgb[100]&&!vrgb[96+3-n]),.outline(edges[3-n]&&vmags[(3-n)*11+:11]>=96),
   .out(colored[(3-n)*24+:24]));
 end endgenerate
 assign support_write[19:16]=4'd0;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin fv<=0;vx<=0;vy<=0;vrgb<=0;vmags<=0;vsmoothed<=0;vdir<=0;
   vleft_sign<=0;vleft_weak<=0;vleft_pair<=0;vleft_hard<=0;end
  else begin
   fv<=send;if(send)begin vx<=pending_x;vy<=pending_y;vrgb<=pending_rgb;vmags<=mags;vsmoothed<=smoothed;vdir<=directions;end
   if(fv)begin vleft_sign<=vdir[2];vleft_weak<=vdir[0];vleft_pair<=vpair[0];vleft_hard<=vdir[1];end
  end
 end
 // Existing centre-crop geometry and synchronous two-bank cache. All modes
 // traverse it, including bypass. Right half never contains cartoon/boxes.
 reg draining,drain_bank,drain_mode;reg [8:0] drain_x;
 wire emit=draining || (fv && vy!=0);
 wire [8:0] ox=draining?drain_x:vx;
 wire omode=draining?drain_mode:vrgb[101];
 wire obank=draining?drain_bank:!vy[0];
 wire [8:0] offset=omode?CROP+(ox>=HALF?ox-HALF:ox):ox;
 wire [99:0] cache_write={edges,colored};wire [99:0] cache_read;
 generate for(n=0;n<5;n=n+1)begin:g_cache
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
 .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),.CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) m(
 .doa(),.dia(cache_write[n*20+:20]),.addra({vy[0],vx}),.cea(fv),.ocea(1'b0),.clka(I_clk),.wea(fv),.rsta(1'b0),.bea(1'b0),
 .dob(cache_read[n*20+:20]),.dib(20'd0),.addrb({obank,offset}),.ceb(emit),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 end endgenerate
 reg ed,sd,ld,bd;integer lane;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   x<=0;y<=0;mode<=0;flush<=0;fx<=0;v1<=0;v2<=0;virtual1<=0;
   x1<=0;x2<=0;y1<=0;y2<=0;bottom1<=0;bottom2<=0;middle2<=0;tl<=0;ml<=0;bl<=0;
   pending<=0;pending_last<=0;pending_x<=0;pending_y<=0;pending_rgb<=0;pmag<=0;psmooth<=0;end_mag<=0;end_smooth<=0;pdir<=0;end_dir<=0;
   draining<=0;drain_x<=0;drain_bank<=0;drain_mode<=0;ed<=0;sd<=0;ld<=0;bd<=0;O_valid<=0;O_start<=0;O_last<=0;O_data<=0;
  end else begin
   if(I_start)begin mode<=I_enable;x<=0;y<=0;end
   if(I_valid&&!flush)begin
    if(rx==BEATS-1)begin x<=0;y<=ry==IMG_HEIGHT-1?0:ry+1'b1;end else begin x<=rx+1'b1;y<=ry;end
    if(rx==BEATS-1 && ry==IMG_HEIGHT-1)begin flush<=1;fx<=0;end
   end
   if(flush)begin if(fx==BEATS-1)begin flush<=0;fx<=0;end else fx<=fx+1'b1;end
   v1<=run;virtual1<=flush;x1<=rx;y1<=ry;bottom1<=gray;
   v2<=v1;x2<=x1;y2<=y1;middle2<=line_out;bottom2<=virtual1?line_out[31:0]:bottom1;
   if(kernel_valid)begin
    tl<=top[15:0];ml<=mid[15:0];bl<=bottom2[15:0];
    pending<=1;pending_last<=x2==BEATS-1;pending_x<=x2;pending_y<=cy;pending_rgb<=middle2[133:32];
    pmag<={mag[1],mag[2],mag[3]};psmooth<={smooth[1],smooth[2],smooth[3]};end_mag<=mag[4];end_smooth<=smooth[4];pdir<={direction[1],direction[2],direction[3]};end_dir<=direction[4];
   end else if(send)pending<=0;
   if(fv && vy==IMG_HEIGHT-1 && vx==BEATS-1)begin
    draining<=1;drain_x<=0;drain_bank<=vy[0];drain_mode<=vrgb[101];
   end
   if(draining)begin if(drain_x==BEATS-1)begin draining<=0;drain_x<=0;end else drain_x<=drain_x+1'b1;end
   ed<=emit;sd<=emit&&!draining&&vy==1&&vx==0;ld<=emit&&ox==BEATS-1;bd<=omode&&ox>=HALF;
   O_valid<=ed;O_start<=sd;O_last<=ld;O_data<=cache_read[95:0];
   if(bd)for(lane=0;lane<4;lane=lane+1)O_data[lane*24+:24]<={24{cache_read[96+lane]}};
  end
 end
endmodule

module visual_row_ram #(parameter WIDTH=36)(input clk,en,we,input [8:0] addr,input [WIDTH-1:0] di,output [WIDTH-1:0] dout);
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),.ADDR_WIDTH_A(9),.ADDR_WIDTH_B(9),
 .DATA_DEPTH_A(512),.DATA_DEPTH_B(512),.MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("READBEFOREWRITE"),.WRITEMODE_B("NORMAL"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),.CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) m(
 .doa(dout),.dia(di),.addra(addr),.cea(en),.ocea(1'b0),.clka(clk),.wea(we),.rsta(1'b0),.bea(1'b0),
 .dob(),.dib({WIDTH{1'b0}}),.addrb(9'd0),.ceb(1'b0),.oceb(1'b0),.clkb(clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
endmodule

// Preserve RGB differences EXACTLY: quantize/smooth luminance, then add a
// single bounded offset to all channels. No forced gray, channel LUT or
// independent saturation. At a gamut boundary the effect weakens safely.
module visual_color_pixel #(parameter OUTLINE_ENABLE=1)(
 input [23:0] rgb,input [7:0] smoothed,input enable,outline,output [23:0] out);
 wire [7:0] r=rgb[23:16],g=rgb[15:8],b=rgb[7:0];
 wire [7:0] lo1=r<g?r:g,hi1=r>g?r:g;
 wire [7:0] lo=lo1<b?lo1:b,hi=hi1>b?hi1:b;
 wire [9:0] ys={2'd0,r}+{1'd0,g,1'b0}+{2'd0,b}+10'd2;
 wire [7:0] y=ys[9:2];
 wire [7:0] q=smoothed==0?8'd0:smoothed==255?8'd255:{smoothed[7:4],4'd8};
 wire [7:0] lift=(q[7]?~q:q)>>2;
 wire [8:0] goal={1'b0,q}+{1'b0,lift};
 wire signed [9:0] delta=$signed({1'b0,goal})-$signed({2'd0,y});
 wire signed [9:0] min_delta=-$signed({2'd0,lo});
 wire signed [9:0] max_delta=$signed({2'd0,~hi});
 wire signed [9:0] bounded=delta<min_delta?min_delta:delta>max_delta?max_delta:delta;
 wire [7:0] rr=$signed({2'd0,r})+bounded,gg=$signed({2'd0,g})+bounded,bb=$signed({2'd0,b})+bounded;
 wire [7:0] ol1=rr<gg?rr:gg,ol=ol1<bb?ol1:bb;
 wire [7:0] ink=OUTLINE_ENABLE!=0 && outline?ol>>1:8'd0;
 assign out=!enable?rgb:{(rr-ink),(gg-ink),(bb-ink)};
endmodule


// Existing banked centre crop, supplied with edges from the SHARED stencil.
module edge_visual_cache_96 #(parameter IMG_WIDTH=1280,IMG_HEIGHT=720)(
 input I_clk,I_rst_n,I_valid,I_start,I_enable,input [95:0] I_data,input [3:0] I_edges,
 output reg O_valid,O_start,O_last,output reg [95:0] O_data);
 localparam BEATS=IMG_WIDTH/4,HALF=BEATS/2,CROP=BEATS/4;
 reg [8:0] x;reg [10:0] y;reg mode;
 wire [8:0] px=I_start?9'd0:x;wire [10:0] py=I_start?11'd0:y;
 wire active=I_start?I_enable:mode;
 reg v1,m1;reg [8:0] x1;reg [10:0] y1;reg [95:0] data1;reg [3:0] edges;
 genvar n;
    reg flushing,flush_bank,flush_mode;reg [8:0] flush_x;
    wire emit=flushing || (v1 && y1!=0);
    wire [8:0] out_x=flushing?flush_x:x1;
    wire out_mode=flushing?flush_mode:m1;
    wire out_bank=flushing?flush_bank:!y1[0];
    wire [8:0] offset=out_mode ? CROP+((out_x>=HALF)?out_x-HALF:out_x):out_x;
    wire [9:0] read_address={out_bank,offset};
    wire [9:0] write_address={y1[0],x1};
    wire [99:0] cache_read;
    wire [95:0] outlined;
    generate for(n=0;n<4;n=n+1)begin:g_outline
        assign outlined[n*24+:24]=data1[n*24+:24];
    end endgenerate
    wire [99:0] cache_write={edges,outlined};
    generate for(n=0;n<5;n=n+1)begin:g_line
        PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),
            .ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),.DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),
            .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
            .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
            .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
            .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
            .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
            .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
            .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory (
            .doa(),.dia(cache_write[n*20+:20]),.addra(write_address),
            .cea(v1),.ocea(1'b0),.clka(I_clk),.wea(v1),.rsta(1'b0),.bea(1'b0),
            .dob(cache_read[n*20+:20]),.dib(20'd0),.addrb(read_address),
            .ceb(emit),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
            .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
    end endgenerate
    reg emit_d,start_d,last_d,binary_d;
    wire first_out=!flushing && y1==1 && x1==0;
    integer lane;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            x<=0;y<=0;mode<=0;v1<=0;m1<=0;x1<=0;y1<=0;data1<=0;edges<=0;
            flushing<=0;flush_x<=0;flush_bank<=0;flush_mode<=0;
            emit_d<=0;start_d<=0;last_d<=0;binary_d<=0;
            O_valid<=0;O_start<=0;O_last<=0;O_data<=0;
        end else begin
            if(I_start)begin mode<=I_enable;x<=0;y<=0;end
            if(I_valid)begin
                if(px==BEATS-1)begin x<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;end
                else begin x<=px+1'b1;y<=py;end
            end
            v1<=I_valid;m1<=active;x1<=px;y1<=py;data1<=I_data;edges<=I_edges;
            if(v1)begin
                
                if(x1==BEATS-1 && y1==IMG_HEIGHT-1)begin
                    flushing<=1;flush_x<=0;flush_bank<=y1[0];flush_mode<=m1;
                end
            end
            if(flushing)begin
                if(flush_x==BEATS-1)begin flushing<=0;flush_x<=0;end
                else flush_x<=flush_x+1'b1;
            end
            emit_d<=emit;start_d<=emit&&first_out;last_d<=emit&&out_x==BEATS-1;
            binary_d<=out_mode&&out_x>=HALF;
            O_valid<=emit_d;O_start<=start_d;O_last<=last_d;O_data<=cache_read[95:0];
            if(binary_d)for(lane=0;lane<4;lane=lane+1)
                O_data[lane*24+:24]<={24{cache_read[96+lane]}};
        end
    end
endmodule


// Shared with the centred mask filter: RGB is already the centre row, and
// I_bottom_gray comes from its existing lookahead. One additional gray row
// supplies the top. No additional RGB line or frame storage is allocated.
module visual_neighborhood_96 #(parameter IMG_WIDTH=1280,IMG_HEIGHT=720)(
 input I_clk,I_rst_n,I_valid,I_start,I_last,I_cartoon_enable,
 input [95:0] I_data,input [31:0] I_bottom_gray,input [3:0] I_mask,I_raw_mask,I_seed_mask,
 output O_valid,O_start,O_last,output [95:0] O_data,output [3:0] O_mask,O_raw_mask,O_seed_mask,O_edges);
 localparam BEATS=IMG_WIDTH/4,OUTLINE_ENABLE=1;
 reg [8:0] x;reg [10:0] y;reg mode;
 wire [8:0] px=I_start?9'd0:x;wire [10:0] py=I_start?11'd0:y;
 wire [31:0] gray;genvar n;
 generate for(n=0;n<4;n=n+1)begin:g_gray
  wire [23:0] p=I_data[(3-n)*24+:24];
  wire [9:0] sum={2'd0,p[23:16]}+{1'd0,p[15:8],1'b0}+{2'd0,p[7:0]}+10'd2;
  assign gray[(3-n)*8+:8]=sum[9:2];
 end endgenerate
 wire [31:0] top2;reg [31:0] mid2,bottom2;reg [108:0] data2;
 reg v2;reg [8:0] x2;reg [10:0] y2;reg [15:0] tl,ml,bl;
 visual_row_ram #(.WIDTH(32)) top_row(.clk(I_clk),.en(I_valid),.we(I_valid),.addr(px),.di(gray),.dout(top2));
 wire [31:0] mid=mid2,top=y2==0?mid2:top2;
 wire kernel_valid=v2;wire [10:0] cy=y2;
  function [7:0] at;
  input [31:0] word4;input [15:0] left2;input integer idx;input first;
  begin
   if(idx<0)at=first?word4[31:24]:(idx==-2?left2[15:8]:left2[7:0]);
   else if(idx>3)at=word4[7:0];else at=word4[31-idx*8-:8];
  end
 endfunction
 reg pending,pending_last;
 wire flush_end=pending&&pending_last;
 wire [10:0] mag[0:3];wire [7:0] smooth[0:3];wire [5:0] direction[0:3];
 generate for(n=0;n<4;n=n+1)begin:g_sobel
  // Kernel0 normally completes the previous beat using the next beat.
  // At EOL it is idle for that job: the saved last two columns instead
  // complete the right border. The same send clock captures the result.
  wire first=x2==0&&!(n==0&&flush_end);
  wire [7:0] a=at(top,tl,n-2,first),b=at(top,tl,n-1,first),c=n==0&&flush_end?tl[7:0]:at(top,tl,n,first);
  wire [7:0] d=at(mid,ml,n-2,first),e=at(mid,ml,n-1,first),f=n==0&&flush_end?ml[7:0]:at(mid,ml,n,first);
  wire [7:0] g=at(bottom2,bl,n-2,first),h=at(bottom2,bl,n-1,first),i=n==0&&flush_end?bl[7:0]:at(bottom2,bl,n,first);
  wire [9:0] right={2'd0,c}+{1'd0,f,1'b0}+{2'd0,i};
  wire [9:0] leftv={2'd0,a}+{1'd0,d,1'b0}+{2'd0,g};
  wire [9:0] down={2'd0,g}+{1'd0,h,1'b0}+{2'd0,i};
  wire [9:0] up={2'd0,a}+{1'd0,b,1'b0}+{2'd0,c};
  wire signed [10:0] diff_x=$signed({1'b0,right})-$signed({1'b0,leftv});
  wire [9:0] dx=diff_x[10]?-diff_x[9:0]:diff_x[9:0];
  wire signed [10:0] diff_y=$signed({1'b0,down})-$signed({1'b0,up});
  wire [9:0] dy=diff_y[10]?-diff_y[9:0]:diff_y[9:0];
  assign mag[n]={1'b0,dx}+{1'b0,dy};
  // Qualified scalar gradients; unlike the old RGB555 test these retain
  // sub-bin slopes and do not require three channel masks to match exactly.
  assign direction[n]={!diff_x[10]&&diff_x!=0,dx>=40,dx>=16,!diff_y[10]&&diff_y!=0,dy>=40,dy>=16};
  // Same 1-2-1 Gaussian, using the Sobel columns already above.
  // Each column <=1020 (10 bits); total <=4088 (12 bits).
  // Grouping is exact, including +8 rounding; no extra cycle or RAM.
  wire [9:0] center_col={2'd0,b}+{1'd0,e,1'b0}+{2'd0,h};
  wire [11:0] gs={2'd0,leftv}+{1'd0,center_col,1'b0}+{2'd0,right}+12'd8;
  assign smooth[n]=gs[11:4];
 end endgenerate

  reg [8:0] pending_x;reg [10:0] pending_y;
 reg [108:0] pending_rgb;
 reg [32:0] pmag;reg [23:0] psmooth;
 reg [17:0] pdir;
 wire send=pending && (pending_last || (kernel_valid && x2!=0));
 wire [43:0] mags={pmag,mag[0]};
 wire [31:0] smoothed={psmooth,smooth[0]};
 wire [23:0] directions={pdir,direction[0]};
 reg fv;reg [8:0] vx;reg [10:0] vy;reg [108:0] vrgb;
 reg [43:0] vmags;reg [31:0] vsmoothed;reg [23:0] vdir;
 wire [19:0] support_read,support_write;
 wire [3:0] edges,hpair,vpair;
 reg vleft_sign,vleft_weak,vleft_pair,vleft_hard;
 // One finite-support row, no recursive weak propagation or frame buffer.
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(20),.DATA_WIDTH_B(20),.ADDR_WIDTH_A(10),.ADDR_WIDTH_B(10),
 .DATA_DEPTH_A(1024),.DATA_DEPTH_B(1024),.MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),.RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),.ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),.CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) support(
 .doa(support_read),.dia(20'd0),.addra({1'b0,pending_x}),.cea(send),.ocea(1'b0),.clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),
 .dob(),.dib(support_write),.addrb({1'b0,vx}),.ceb(fv),.oceb(1'b0),.clkb(I_clk),.web(fv),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
 wire [95:0] colored;
 generate for(n=0;n<4;n=n+1)begin:g_color
  wire [5:0] code=vdir[(3-n)*6+:6];
  wire hs=code[5],hh=code[4],hw=code[3],vs=code[2],vh=code[1],vw=code[0];
  wire [3:0] above_code=support_read[(3-n)*4+:4];
  wire hmatch=hs==above_code[3];
  assign hpair[3-n]=vy!=0 && hw && above_code[1] && hmatch && (hh||above_code[2]);
  wire ps,pw,pp,ph;
  if(n==0)begin assign ps=vleft_sign;assign pw=vx!=0&&vleft_weak;assign pp=vx!=0&&vleft_pair;assign ph=vleft_hard;end
  else begin assign ps=vdir[(4-n)*6+2];assign pw=vdir[(4-n)*6];assign pp=vpair[4-n];assign ph=vdir[(4-n)*6+1];end
  assign vpair[3-n]=vw && pw && vs==ps && (vh||ph);
  assign edges[3-n]=(vy>=2 && hw && above_code[0] && hmatch) || (vw && pp && vs==ps) ||
    (vmags[(3-n)*11+:11]>=160 && ((vy!=0&&hh&&above_code[2]) || ((vx!=0||n!=0)&&vh&&ph)));
  assign support_write[(3-n)*4+:4]={hs,hh,hw,hpair[3-n]};
  visual_color_pipe #(.OUTLINE_ENABLE(OUTLINE_ENABLE)) p(
   .I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(fv),.rgb(vrgb[(3-n)*24+:24]),.smoothed(vsmoothed[(3-n)*8+:8]),
   .enable(vrgb[108]),.outline(edges[3-n]&&vmags[(3-n)*11+:11]>=96),
   .out(colored[(3-n)*24+:24]));
 end endgenerate
 assign support_write[19:16]=4'd0;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin fv<=0;vx<=0;vy<=0;vrgb<=0;vmags<=0;vsmoothed<=0;vdir<=0;
   vleft_sign<=0;vleft_weak<=0;vleft_pair<=0;vleft_hard<=0;end
  else begin
   fv<=send;if(send)begin vx<=pending_x;vy<=pending_y;vrgb<=pending_rgb;vmags<=mags;vsmoothed<=smoothed;vdir<=directions;end
   if(fv)begin vleft_sign<=vdir[2];vleft_weak<=vdir[0];vleft_pair<=vpair[0];vleft_hard<=vdir[1];end
  end
 end

 reg cv;reg [8:0] cx;reg [10:0] row;reg [3:0] cmask,craw,cseed,cedge;
 assign O_valid=cv;assign O_start=cv && cx==0 && row==0;assign O_last=cv && cx==BEATS-1;
 assign O_data=colored;assign O_mask=cmask;assign O_raw_mask=craw;assign O_seed_mask=cseed;assign O_edges=cedge;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   x<=0;y<=0;mode<=0;v2<=0;x2<=0;y2<=0;mid2<=0;bottom2<=0;data2<=0;tl<=0;ml<=0;bl<=0;
   pending<=0;pending_last<=0;pending_x<=0;pending_y<=0;pending_rgb<=0;pmag<=0;psmooth<=0;pdir<=0;
   cv<=0;cx<=0;row<=0;cmask<=0;craw<=0;cseed<=0;cedge<=0;
  end else begin
   if(I_start)begin x<=0;y<=0;mode<=I_cartoon_enable;end
   if(I_valid)begin if(px==BEATS-1)begin x<=0;y<=py==IMG_HEIGHT-1?0:py+1'b1;end else begin x<=px+1'b1;y<=py;end end
   v2<=I_valid;x2<=px;y2<=py;mid2<=gray;bottom2<=I_bottom_gray;
   data2<={(I_start?I_cartoon_enable:mode),I_seed_mask,I_raw_mask,I_mask,I_data};
   if(kernel_valid)begin
    tl<=top[15:0];ml<=mid[15:0];bl<=bottom2[15:0];
    pending<=1;pending_last<=x2==BEATS-1;pending_x<=x2;pending_y<=cy;pending_rgb<=data2;
    pmag<={mag[1],mag[2],mag[3]};psmooth<={smooth[1],smooth[2],smooth[3]};pdir<={direction[1],direction[2],direction[3]};
   end else if(send)pending<=0;
   cv<=fv;if(fv)begin cx<=vx;row<=vy;cmask<=vrgb[99:96];craw<=vrgb[103:100];cseed<=vrgb[107:104];cedge<=edges;end
  end
 end
endmodule

// Register the bounded common offset before the final channel sums. This
// breaks the luminance/min/max/outline chain without using a DSP or RAM.
module visual_color_pipe #(parameter OUTLINE_ENABLE=1)(
 input I_clk,I_rst_n,I_valid,input [23:0] rgb,input [7:0] smoothed,input enable,outline,output [23:0] out);
  wire [7:0] r=rgb[23:16],g=rgb[15:8],b=rgb[7:0];
 wire [7:0] lo1=r<g?r:g,hi1=r>g?r:g;
 wire [7:0] lo=lo1<b?lo1:b,hi=hi1>b?hi1:b;
 wire [9:0] ys={2'd0,r}+{1'd0,g,1'b0}+{2'd0,b}+10'd2;
 wire [7:0] y=ys[9:2];
 wire [7:0] q=smoothed==0?8'd0:smoothed==255?8'd255:{smoothed[7:4],4'd8};
 wire [7:0] lift=(q[7]?~q:q)>>2;
 wire [8:0] goal={1'b0,q}+{1'b0,lift};
 wire signed [9:0] delta=$signed({1'b0,goal})-$signed({2'd0,y});
 wire signed [9:0] min_delta=-$signed({2'd0,lo});
 wire signed [9:0] max_delta=$signed({2'd0,~hi});
 wire signed [9:0] bounded=delta<min_delta?min_delta:delta>max_delta?max_delta:delta;

 reg [23:0] p;reg [7:0] off,low;reg enabled,ink_on;
 always @(posedge I_clk or negedge I_rst_n)
  if(!I_rst_n)begin p<=0;off<=0;low<=0;enabled<=0;ink_on<=0;end
  else if(I_valid)begin p<=rgb;off<=bounded[7:0];low<=lo+bounded[7:0];enabled<=enable;ink_on<=outline;end
 wire [7:0] ink=OUTLINE_ENABLE!=0 && ink_on?low>>1:8'd0;
 wire [7:0] change=off-ink;
 wire [7:0] rr=p[23:16]+change,gg=p[15:8]+change,bb=p[7:0]+change;
 assign out=enabled?{rr,gg,bb}:p;
endmodule
