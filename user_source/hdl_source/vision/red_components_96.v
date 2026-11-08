`timescale 1ns/1ps
// Run-length CCL, exact four-connectivity; one shared statistics engine.
// Sorted interval matching; FIFO_DEPTH=48, ROW_RUNS<=64, LABELS<=32.
module red_components_96 #(
    parameter IMG_WIDTH=1280, IMG_HEIGHT=720, MIN_AREA=100,
    parameter LABELS=32, TARGETS=4, ROW_RUNS=64, MIN_SEED_RUN=4, MIN_SPAN=4, MIN_BOX_AREA=0, ROW_DESCRIPTOR_ERAM=1, CB=(TARGETS<8?3:4), STATS_DRAM=0,SEEDED=0,STRONG_START=0,SEED_COUNTS=0,MOTION_EDGE_GUARD=0,EVENT_MIN_RUN=1,TARGET_MIN_SEEDS=0,MOTION_MIN_AREA=0,MOTION_MIN_BOX_AREA=0,CACHE_GEOMETRY=0,FAST_QUEUE_LIMIT=0,FAST_EVENT_COUNT=0,FAST_EVENT_PAYLOAD=0,SHARE_LAST_ROW=0,CACHE_SEEDS=0
) (
    input I_clk,I_rst_n,I_valid,I_frame_start,
    input [3:0] I_mask,
    output [15:0] O_pixel_x,O_pixel_y,
    output reg [CB-1:0] O_count,
    output reg O_limit,
    output [4:0] O_limit_reason,
    output reg [TARGETS*16-1:0] O_xmin,O_xmax,O_ymin,O_ymax,
    output reg [7:0] O_row_peak,
    output reg [5:0] O_labels,O_objects,
    output reg [15:0] O_dropped,
    output reg O_result_valid,
    output reg [4:0] O_frame_reason,input [3:0] I_seed_mask,input I_motion_only
);
    localparam IDLE=0, MATCH=1, MERGE=2, ADD=3, SELECT=4;
    function integer bits_for;
        input integer size;
        integer v;
        begin v=size-1;bits_for=0;while(v>0) begin bits_for=bits_for+1;v=v>>1;end
            if(bits_for==0) bits_for=1;
        end
    endfunction
    localparam XB=bits_for(IMG_WIDTH),YB=bits_for(IMG_HEIGHT), RB=bits_for(ROW_RUNS+1);
    // Difference stripes are smaller than a solid colour body. Keep them
    // until bounded rectangle clustering, rather than filling static edges.
    wire [19:0] area_floor=I_motion_only&&MOTION_MIN_AREA!=0?MOTION_MIN_AREA:MIN_AREA;
    wire [31:0] box_floor=I_motion_only&&MOTION_MIN_BOX_AREA!=0?MOTION_MIN_BOX_AREA:MIN_BOX_AREA;
    reg [XB-1:0] x;
    reg [YB-1:0] y;
    reg sof_d;
    wire sof=I_frame_start && !sof_d;
    assign O_pixel_x=sof?16'd0:x;
    assign O_pixel_y=sof?16'd0:y;
    reg [1:0] bank;
    wire [1:0] cb=sof?2'd0:bank;
    reg run_open,open_d;reg [2:0] run_seed,seed_d;reg [2:0] closed_seed;reg event_seed;reg [2:0] event_seed_count;reg [4:0] seed_hits[0:LABELS];reg [2:0] closed_count[0:2];reg [LABELS:0] seeded;
    reg [XB-1:0] run_start,run_end,start_d,end_d,closed0[0:2],closed1[0:2];
    integer closes,k,pp;
    always @* begin
        open_d=sof?0:run_open;start_d=sof?0:run_start;end_d=sof?0:run_end;
        closes=0;pp=0;seed_d=sof?0:run_seed;closed_seed=0;
        for(k=0;k<3;k=k+1) begin closed0[k]=0;closed1[k]=0;closed_count[k]=0;end
        for(k=0;k<4;k=k+1) if(I_valid) begin
            pp=O_pixel_x+k;
            if(I_mask[3-k]) begin
                if(!open_d) begin open_d=1;start_d=pp;seed_d=0;end
                if(I_seed_mask[3-k] && seed_d<4)seed_d=seed_d+1'b1;
                end_d=pp;
            end else if(open_d) begin
                if(end_d-start_d+1>=((MOTION_EDGE_GUARD!=0&&I_motion_only)?1:EVENT_MIN_RUN))begin
                  closed0[closes]=start_d;closed1[closes]=end_d;closed_seed[closes]=seed_d>=4;closed_count[closes]=seed_d;closes=closes+1;
                end
                open_d=0;
            end
            if(pp==IMG_WIDTH-1 && open_d) begin
                if(end_d-start_d+1>=((MOTION_EDGE_GUARD!=0&&I_motion_only)?1:EVENT_MIN_RUN))begin
                  closed0[closes]=start_d;closed1[closes]=end_d;closed_seed[closes]=seed_d>=4;closed_count[closes]=seed_d;closes=closes+1;
                end
                open_d=0;
            end
        end
    end
    // Independent closure count: the event payload compactor above keeps its
    // exact ordering, while queue control bypasses its dynamic index chain.
    // Native min-run is 2, or 1 in motion mode. Legacy/general cases fall back.
    wire count_short=(EVENT_MIN_RUN==1) || (MOTION_EDGE_GUARD!=0 && I_motion_only);
    wire count_open=run_open && !sof;
    wire count_prior_long=run_start!=run_end;
    wire [4:0] close_bits;
    assign close_bits[0]=count_open && !I_mask[3] && (count_short || count_prior_long);
    assign close_bits[1]=I_mask[3] && !I_mask[2] && (count_short || count_open);
    assign close_bits[2]=I_mask[2] && !I_mask[1] && (count_short || I_mask[3]);
    assign close_bits[3]=I_mask[1] && !I_mask[0] && (count_short || I_mask[2]);
    assign close_bits[4]=I_mask[0] && (O_pixel_x==IMG_WIDTH-4) && (count_short || I_mask[1]);
    wire [1:0] close_pair0={1'b0,close_bits[0]}+{1'b0,close_bits[1]};
    wire [1:0] close_pair1={1'b0,close_bits[2]}+{1'b0,close_bits[3]};
    wire [2:0] direct_closes={1'b0,close_pair0}+{1'b0,close_pair1}+{2'b0,close_bits[4]};
    wire [2:0] event_closes=(FAST_EVENT_COUNT!=0 && IMG_WIDTH%4==0 &&
        (EVENT_MIN_RUN==1 || EVENT_MIN_RUN==2)) ? (I_valid?direct_closes:3'd0) : closes;
    // Payload candidate: independent records and bounded one-hot compaction.
    // General/legacy parameters retain the original parser and its cycles.
    wire [3*XB-1:0] event_x0,event_x1;
    wire [8:0] event_seed_counts;
    wire [2:0] event_seed_flags;
    generate if(FAST_EVENT_PAYLOAD!=0 && IMG_WIDTH%4==0 &&
        (EVENT_MIN_RUN==1 || EVENT_MIN_RUN==2))begin:g_parallel_payload
        rle_payload_parallel #(.XB(XB)) records(
            .valid(I_valid),.sof(sof),.opened(run_open),.short_run(count_short),
            .last_word(O_pixel_x==IMG_WIDTH-4),.xpos(O_pixel_x[XB-1:0]),
            .prior_start(run_start),.prior_end(run_end),.prior_seed(run_seed),
            .mask(I_mask),.seeds(I_seed_mask),
            .closed0(event_x0),.closed1(event_x1),
            .closed_count(event_seed_counts),.closed_seed(event_seed_flags),.count());
    end else begin:g_legacy_payload
        assign event_x0={closed0[2],closed0[1],closed0[0]};
        assign event_x1={closed1[2],closed1[1],closed1[0]};
        assign event_seed_counts={closed_count[2],closed_count[1],closed_count[0]};
        assign event_seed_flags=closed_seed;
    end endgenerate
    // Bank-striped FIFO order ensures previous-row labels are complete before matching.
    localparam ROW_BITS=2*XB+6, ROW_AW=bits_for(3*ROW_RUNS);
    wire [ROW_BITS-1:0] row_read;
    wire [XB-1:0] row_read_x0=row_read[0+:XB],row_read_x1=row_read[XB+:XB];
    wire [5:0] row_read_label=row_read[2*XB+:6];
    reg [RB-1:0] row_count[0:2];
    localparam EVENT_BITS=2*XB+YB+RB+2+(SEEDED!=0?(SEED_COUNTS!=0?3:1):0);
    wire [EVENT_BITS-1:0] event_head;
    wire [XB-1:0] qx0_head=event_head[0 +:XB];
    wire [XB-1:0] qx1_head=event_head[XB +:XB];
    wire [YB-1:0] qy_head=event_head[2*XB +:YB];
    wire [RB-1:0] qs_head=event_head[2*XB+YB +:RB];
    wire [1:0] qb_head=event_head[2*XB+YB+RB +:2];
    wire [2:0] qseed_count=SEEDED==0?3'd4:SEED_COUNTS!=0?event_head[EVENT_BITS-1-:3]:{event_head[EVENT_BITS-1],2'b00};
    wire qseed_head=qseed_count>=4;
    wire [6:0] queued;
    reg [2:0] state;
    reg frame_limit,frame_pending,scanning;
    wire pop=!sof && !scanning && state==IDLE && queued!=0;
    reg [XB-1:0] ex0,ex1;reg [YB-1:0] ey;
    reg [1:0] eb;reg [RB-1:0] es;
    integer accepted,slots,free_slots;
    reg enqueue_limit;
    always @* begin
        slots=(sof || O_pixel_x==0)?0:row_count[cb];
        free_slots=48-(sof?0:queued)+(pop?1:0);
        // At most three runs close in a four-pixel word. Parallel constant
        // limits preserve the same-cycle FIFO pop credit, without serial min.
        if(FAST_QUEUE_LIMIT!=0 && ROW_RUNS>=3) begin
            if(event_closes>=3 && slots<=ROW_RUNS-3 && (sof || queued<=45 || (pop && queued==46))) accepted=3;
            else if(event_closes>=2 && slots<=ROW_RUNS-2 && (sof || queued<=46 || (pop && queued==47))) accepted=2;
            else if(event_closes>=1 && slots<=ROW_RUNS-1 && (sof || queued<=47 || (pop && queued==48))) accepted=1;
            else accepted=0;
        end else begin
            accepted=event_closes;
            if(accepted>ROW_RUNS-slots) accepted=ROW_RUNS-slots;
            if(accepted>free_slots) accepted=free_slots;
        end
        enqueue_limit=accepted<event_closes;
        // Row-bank reuse deadline: do not silently return inaccurate counts.
        if(I_valid && O_pixel_x==0 && O_pixel_y>=3) begin
            if(state!=IDLE && ey<O_pixel_y-1) enqueue_limit=1;
            if(queued!=0 && qy_head<O_pixel_y-1) enqueue_limit=1;
        end
    end
    wire [RB-1:0] slot0=slots,slot1=slots+1,slot2=slots+2;
    rle_event_fifo #(.FAST_BANKS(FAST_EVENT_COUNT),.WIDTH(EVENT_BITS)) u_event_fifo (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(sof),.I_pop(pop),
        .I_count(accepted[1:0]),
        .I_data0({(SEED_COUNTS!=0?event_seed_counts[0+:3]:{2'd0,event_seed_flags[0]}),cb,slot0,O_pixel_y[YB-1:0],event_x1[0+0*XB+:XB],event_x0[0*XB+:XB]}),
        .I_data1({(SEED_COUNTS!=0?event_seed_counts[3+:3]:{2'd0,event_seed_flags[1]}),cb,slot1,O_pixel_y[YB-1:0],event_x1[0+1*XB+:XB],event_x0[1*XB+:XB]}),
        .I_data2({(SEED_COUNTS!=0?event_seed_counts[6+:3]:{2'd0,event_seed_flags[2]}),cb,slot2,O_pixel_y[YB-1:0],event_x1[0+2*XB+:XB],event_x0[2*XB+:XB]}),
        .O_head(event_head),.O_count(queued)
    );
    reg [5:0] parent[0:LABELS];
    reg [19:0] area[0:LABELS];
    reg [XB-1:0] xmin[0:LABELS],xmax[0:LABELS];
    reg [YB-1:0] ymin[0:LABELS],ymax[0:LABELS];
    reg [YB-1:0] last_ref[0:LABELS];
    reg [LABELS:0] span_ok,stat_valid,large_area;
    reg [6:0] allocated;
    reg [5:0] root;
    reg [LABELS:0] pending;
    reg [5:0] overlap_root,merge_root,recycle_root;
    reg [RB-1:0] row_scan,row_floor;
    reg [YB-1:0] match_y;
    wire seed_small=(ex1-ex0+1)<MIN_SEED_RUN;
    reg [1:0] pb;
    integer r,ri;
    always @* begin
        pb=(eb==0)?2:eb-1;overlap_root=0;ri=pb*ROW_RUNS+row_scan;
        // A single row-descriptor read and parent lookup, rather than 16
        // parallel memory multiplexers. Input video never waits for this scan.
        if(ey!=0 && row_scan<row_count[pb] && row_read_x0<=ex1 && row_read_x1>=ex0)
            if(row_read_label!=0) overlap_root=parent[row_read_label];
        merge_root=0;
        for(r=LABELS;r>=1;r=r-1) begin
            if(pending[r]) merge_root=r;
        end
        recycle_root=0;
        // An alias is reusable only when no current/previous-row descriptor
        // references its SLOT. Root age alone does not prove alias safety.
        for(r=LABELS;r>=1;r=r-1) begin
            if(ey>1 && (SHARE_LAST_ROW!=0 ?
                (ymax[r]<ey-1 && (parent[r]!=r ||
                 ((STATS_DRAM!=0?!large_area[r]:area[r]<area_floor) || !span_ok[r]))) :
                ((parent[r]!=r && last_ref[r]<ey-1) ||
                 (parent[r]==r && ymax[r]<ey-1 &&
                  ((STATS_DRAM!=0?!large_area[r]:area[r]<area_floor) || !span_ok[r])))))
                recycle_root=r;
        end
    end
    // Preload the first predecessor descriptor at FIFO pop. Each MATCH
    // clock requests its successor, so synchronous RAM adds no scan cycles.
    wire [1:0] preload_bank=(qb_head==0)?2'd2:qb_head-1'b1;
    wire [RB-1:0] preload_scan=(qy_head!=match_y)?{RB{1'b0}}:row_floor;
    wire [ROW_AW-1:0] row_ra=pop?preload_bank*ROW_RUNS+preload_scan:pb*ROW_RUNS+row_scan+1'b1;
    wire [ROW_AW-1:0] row_now=pb*ROW_RUNS+row_scan;
    wire [ROW_AW-1:0] row_wa=pop?qb_head*ROW_RUNS+qs_head:eb*ROW_RUNS+es;
    wire row_we=I_rst_n && !sof && (pop || state==ADD);
    wire [ROW_BITS-1:0] row_wd=pop?{6'd0,qx1_head,qx0_head}:{root,ex1,ex0};
    generate if(ROW_DESCRIPTOR_ERAM!=0)begin:g_row_eram
        PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(ROW_BITS),.DATA_WIDTH_B(ROW_BITS),
            .ADDR_WIDTH_A(ROW_AW),.ADDR_WIDTH_B(ROW_AW),
            .DATA_DEPTH_A(2**ROW_AW),.DATA_DEPTH_B(2**ROW_AW),
            .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
            .WRITEMODE_A("NORMAL"),.WRITEMODE_B("NORMAL"),
            .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
            .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
            .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
            .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
            .CLKMODE("SYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) row_ram(
            .doa(),.dia(row_wd),.addra(row_wa),.cea(row_we),.ocea(1'b0),.clka(I_clk),.wea(row_we),.rsta(1'b0),.bea(1'b0),
            .dob(row_read),.dib({ROW_BITS{1'b0}}),.addrb(row_ra),.ceb(1'b1),.oceb(1'b0),.clkb(I_clk),.web(1'b0),.rstb(1'b0),.beb(1'b0),
            .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
    end else begin:g_row_distributed
        reg [ROW_BITS-1:0] descriptors[0:3*ROW_RUNS-1];
        always @(posedge I_clk)if(row_we)descriptors[row_wa]<=row_wd;
        assign row_read=descriptors[row_now];
    end endgenerate
    reg [6:0] scan;
    reg [2:0] phase;
    reg [6:0] outer;
    reg [LABELS:0] eligible;
    // Streaming statistics and frame-end filtering never run together.
    // Share their two read selectors instead of inferring four wide muxes.
    wire [5:0] stat_a=scanning?scan[5:0]:root;
    wire [5:0] stat_b=scanning?outer[5:0]:merge_root;
    // Cache dimensions on the existing statistics write; no extra clocks.
    // 24 labels use32 words, while legacy32-label mode retains64 words.
    localparam GEOMETRY_CACHE=CACHE_GEOMETRY!=0 && STATS_DRAM!=0;
    localparam STAT_BASE=20+2*XB+YB;
    localparam SEED_CACHE=CACHE_SEEDS!=0&&STATS_DRAM!=0&&SEEDED!=0;
    localparam STAT_CORE_W=STAT_BASE+(GEOMETRY_CACHE?(XB+1+YB+1):0);
    localparam STAT_W=STAT_CORE_W+(SEED_CACHE?6:0);
    localparam STAT_AW=GEOMETRY_CACHE?bits_for(LABELS+1):6;
    wire [STAT_W-1:0] stat_read_a,stat_read_b;
    // These fields have the SAME two read selectors and the SAME one write
    // as area/geometry. Invalidated aliases need no second RAM clear port.
    wire [4:0] seed_hits_a=SEED_CACHE?(stat_valid[stat_a]?stat_read_a[STAT_CORE_W+:5]:5'd0):seed_hits[stat_a];
    wire [4:0] seed_hits_b=SEED_CACHE?(stat_valid[stat_b]?stat_read_b[STAT_CORE_W+:5]:5'd0):seed_hits[stat_b];
    wire seeded_a=SEED_CACHE?(stat_valid[stat_a]&&stat_read_a[STAT_CORE_W+5]):seeded[stat_a];
    wire seeded_b=SEED_CACHE?(stat_valid[stat_b]&&stat_read_b[STAT_CORE_W+5]):seeded[stat_b];
    wire [5:0] seed_sum={1'b0,seed_hits_a}+(state==MERGE?{1'b0,seed_hits_b}:{3'd0,event_seed_count});
    wire [4:0] next_seed_hits=seed_sum>31?5'd31:seed_sum[4:0];
    wire next_seeded=seeded_a|(state==MERGE?seeded_b:event_seed);
    wire [4:0] candidate_seeds=SEED_CACHE?seed_hits_a:seed_hits[scan];
    wire candidate_seeded=SEED_CACHE?seeded_a:seeded[scan];
    wire [19:0] area_a=STATS_DRAM!=0?(stat_valid[stat_a]?stat_read_a[0+:20]:20'd0):area[stat_a];
    wire [19:0] area_b=STATS_DRAM!=0?(stat_valid[stat_b]?stat_read_b[0+:20]:20'd0):area[stat_b];
    wire [XB-1:0] xmin_a=STATS_DRAM!=0?(stat_valid[stat_a]?stat_read_a[20+:XB]:{XB{1'b1}}):xmin[stat_a];
    wire [XB-1:0] xmax_a=STATS_DRAM!=0?(stat_valid[stat_a]?stat_read_a[20+XB+:XB]:{XB{1'b0}}):xmax[stat_a];
    wire [XB-1:0] xmin_b=STATS_DRAM!=0?(stat_valid[stat_b]?stat_read_b[20+:XB]:{XB{1'b1}}):xmin[stat_b];
    wire [XB-1:0] xmax_b=STATS_DRAM!=0?(stat_valid[stat_b]?stat_read_b[20+XB+:XB]:{XB{1'b0}}):xmax[stat_b];
    wire [YB-1:0] ymin_a=STATS_DRAM!=0?(stat_valid[stat_a]?stat_read_a[20+2*XB+:YB]:{YB{1'b1}}):ymin[stat_a];
    wire [YB-1:0] ymin_b=STATS_DRAM!=0?(stat_valid[stat_b]?stat_read_b[20+2*XB+:YB]:{YB{1'b1}}):ymin[stat_b];
    wire [YB-1:0] ymax_a=ymax[stat_a],ymax_b=ymax[stat_b];
    wire [XB-1:0] update_x0=(state==MERGE)?xmin_b:ex0;
    wire [XB-1:0] update_x1=(state==MERGE)?xmax_b:ex1;
    wire [YB-1:0] update_y0=(state==MERGE)?ymin_b:ey;
    wire [YB-1:0] update_y1=(state==MERGE)?ymax_b:ey;
    wire [XB-1:0] next_x0=(update_x0<xmin_a)?update_x0:xmin_a;
    wire [XB-1:0] next_x1=(update_x1>xmax_a)?update_x1:xmax_a;
    wire [YB-1:0] next_y0=(update_y0<ymin_a)?update_y0:ymin_a;
    wire [YB-1:0] next_y1=(update_y1>ymax_a)?update_y1:ymax_a;
    wire [XB:0] next_width={1'b0,next_x1}-{1'b0,next_x0}+1'b1;
    wire [YB:0] next_height={1'b0,next_y1}-{1'b0,next_y0}+1'b1;
    wire next_span_ok=next_width>=MIN_SPAN && next_height>=MIN_SPAN;
    // Two asynchronous reads, ONE shared write. Replicate only the small
    // distributed memory, not registers and wide read muxes. The alias-valid
    // bit invalidates merged/recycled words; reset never touches RAM content.
    // Keep ymax/large-area summaries in registers for parallel label recycling.
    wire stat_we=I_rst_n && !sof && !scanning &&
        (state==ADD || (state==MERGE && merge_root!=0));
    wire [19:0] stat_new_area=state==MERGE?area_a+area_b:area_a+(ex1-ex0+1'b1);
    wire [STAT_W-1:0] stat_write;
    wire [STAT_CORE_W-1:0] stat_core_write;
    generate if(SEED_CACHE)begin:g_seed_write assign stat_write={next_seeded,next_seed_hits,stat_core_write};
    end else begin:g_no_seed_write assign stat_write=stat_core_write;end endgenerate
    generate if(GEOMETRY_CACHE)begin:g_write_geometry
        assign stat_core_write={next_height,next_width,next_y0,next_x1,next_x0,stat_new_area};
    end else begin:g_write_legacy
        assign stat_core_write={next_y0,next_x1,next_x0,stat_new_area};
    end endgenerate
    genvar stat_port;
    generate if(STATS_DRAM!=0)begin:g_stat_dram
        for(stat_port=0;stat_port<2;stat_port=stat_port+1)begin:g_read_copy
            wire [STAT_W-1:0] rd;
            wire [5:0] ra=stat_port==0?stat_a:stat_b;
            PH1P_LOGIC_DRAM #(.DATA_WIDTH_W(STAT_W),.DATA_WIDTH_R(STAT_W),
                .ADDR_WIDTH_W(STAT_AW),.ADDR_WIDTH_R(STAT_AW),.DATA_DEPTH_W(2**STAT_AW),.DATA_DEPTH_R(2**STAT_AW),
                .READREG("DISABLE")) memory(
                .di(stat_write),.waddr(root[STAT_AW-1:0]),.we(stat_we),.wclk(I_clk),
                .rclk(I_clk),.rrst(1'b0),.rce(1'b0),.raddr(ra[STAT_AW-1:0]),.rdoq(),.rdo(rd));
            if(stat_port==0)assign stat_read_a=rd;
            else assign stat_read_b=rd;
        end
    end else begin:g_stat_registers
        assign stat_read_a=0;assign stat_read_b=0;
    end endgenerate
    reg [19:0] best_score;
    reg [5:0] best_label;
    reg [XB-1:0] best_x;
    reg [TARGETS*16-1:0] wx0,wx1,wy0,wy1;
    reg [CB-1:0] sort_step,sort_pass;
    wire [CB-1:0] sort_a=sort_step,sort_b=sort_step+1'b1;
    integer t;
    wire [XB+YB+1:0] outer_box;
    wire [XB:0] bound_w_a,bound_w_b;
    wire [YB:0] bound_h_a,bound_h_b;
    generate if(GEOMETRY_CACHE)begin:g_read_geometry
        assign bound_w_a=stat_read_a[STAT_BASE+:XB+1];
        assign bound_w_b=stat_read_b[STAT_BASE+:XB+1];
        assign bound_h_a=stat_read_a[STAT_BASE+XB+1+:YB+1];
        assign bound_h_b=stat_read_b[STAT_BASE+XB+1+:YB+1];
    end else begin:g_read_legacy
        assign bound_w_a={1'b0,xmax_a}-{1'b0,xmin_a}+1'b1;
        assign bound_w_b={1'b0,xmax_b}-{1'b0,xmin_b}+1'b1;
        assign bound_h_a={1'b0,ymax_a}-{1'b0,ymin_a}+1'b1;
        assign bound_h_b={1'b0,ymax_b}-{1'b0,ymin_b}+1'b1;
    end endgenerate
    wire candidate_valid=(SEEDED==0 || ((MOTION_EDGE_GUARD!=0 && I_motion_only)?candidate_seeds>=8:(candidate_seeded && (TARGET_MIN_SEEDS==0 || candidate_seeds>=TARGET_MIN_SEEDS)))) && outer_box>=box_floor && parent[scan]==scan && area_a>=area_floor &&
        xmin_a<=xmax_a && ymin_a<=ymax_a &&
        xmax_a<IMG_WIDTH && ymax_a<IMG_HEIGHT &&
        (GEOMETRY_CACHE ? (bound_w_a>=MIN_SPAN && bound_h_a>=MIN_SPAN) :
        (xmax_a-xmin_a+1>=MIN_SPAN && ymax_a-ymin_a+1>=MIN_SPAN));
    wire [XB:0] outer_w=scanning&&phase==0 ?
        bound_w_a : bound_w_b;
    wire [YB:0] outer_h=scanning&&phase==0 ?
        bound_h_a : bound_h_b;
    assign outer_box=outer_w*outer_h;
    // Conservative duplicate suppression for separate solid cards only:
    // fully contained, <=1/8 foreground area, inside a >=75%-filled body.
    // Partial overlap and sparse/triangular outer boxes never suppress.
    wire nested_small=outer!=scan && eligible[outer] &&
        xmin_b<=xmin_a && xmax_b>=xmax_a &&
        ymin_b<=ymin_a && ymax_b>=ymax_a &&
        area_a<=(area_b>>3) &&
        area_b>=outer_box-(outer_box>>2);
    reg [CB-1:0] work_count;
    reg [7:0] row_total,row_peak;
    reg [15:0] dropped;
    reg [4:0] frame_reason;
    reg [5:0] object_total;
    wire [8:0] row_next={1'b0,((sof || O_pixel_x==0)?8'd0:row_total)}+closes;
    wire [16:0] drop_next={1'b0,dropped}+closes-accepted;
    reg work_limit;
    integer j;
    // Independent sticky causes: OBJ, LABEL, ROW, QUEUE, TIME.
    // Hold each cause for 60 subsequent frame starts (about 1 s at 60 fps).
    reg [5:0] reason_hold[0:4];
    wire [4:0] reason_event;
    assign reason_event[0]=scanning && phase==2 && work_count==0 && scan<=LABELS && eligible[scan] && object_total>=TARGETS;
    assign reason_event[1]=!sof && state==SELECT && merge_root==0 && !seed_small && (STRONG_START==0 || (MOTION_EDGE_GUARD!=0&&I_motion_only?event_seed_count!=0:event_seed)) && allocated>LABELS && recycle_root==0;
    assign reason_event[2]=I_valid && closes>ROW_RUNS-slots;
    assign reason_event[3]=I_valid && closes>free_slots;
    assign reason_event[4]=(sof && (frame_pending || scanning || state!=IDLE || queued!=0)) ||
        (I_valid && O_pixel_x==0 && O_pixel_y>=3 &&
        ((state!=IDLE && ey<O_pixel_y-1) || (queued!=0 && qy_head<O_pixel_y-1)));
    genvar cause;
    generate for(cause=0;cause<5;cause=cause+1) begin:g_reason
        assign O_limit_reason[cause]=reason_hold[cause]!=0;
        always @(posedge I_clk or negedge I_rst_n)
            if(!I_rst_n) reason_hold[cause]<=0;
            else if(reason_event[cause]) reason_hold[cause]<=60;
            else if(sof && reason_hold[cause]!=0) reason_hold[cause]<=reason_hold[cause]-1'b1;
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x<=0;y<=0;bank<=0;sof_d<=0;run_seed<=0;seeded<=0;event_seed<=0;event_seed_count<=0;run_open<=0;run_start<=0;run_end<=0;
            state<=IDLE;allocated<=1;root<=0;pending<=0;row_scan<=0;row_floor<=0;match_y<=0;
            frame_limit<=0;frame_pending<=0;scanning<=0;scan<=1;
            phase<=0;outer<=1;eligible<=0;sort_step<=0;sort_pass<=0;span_ok<=0;stat_valid<=0;large_area<=0;
            best_score<=0;best_label<=0;best_x<=0;
            ex0<=0;ex1<=0;ey<=0;eb<=0;es<=0;
            work_count<=0;work_limit<=0;wx0<=0;wx1<=0;wy0<=0;wy1<=0;
            O_result_valid<=0;O_count<=0;O_limit<=0;O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;
            row_total<=0;row_peak<=0;dropped<=0;frame_reason<=0;object_total<=0;
            O_row_peak<=0;O_labels<=0;O_objects<=0;O_dropped<=0;O_frame_reason<=0;
            for(j=0;j<3;j=j+1) row_count[j]<=0;
            for(j=0;j<=LABELS;j=j+1) begin
                parent[j]<=j;if(SHARE_LAST_ROW==0)last_ref[j]<=0;ymax[j]<=0;if(!SEED_CACHE)seed_hits[j]<=0;
                if(STATS_DRAM==0)begin area[j]<=0;xmin[j]<={XB{1'b1}};xmax[j]<=0;ymin[j]<={YB{1'b1}};end
            end
        end else begin
            O_result_valid<=0;
            sof_d<=I_frame_start;
            if(sof) begin
                row_total<=I_valid?(row_next[8]?8'hff:row_next[7:0]):0;
                row_peak<=I_valid?(row_next[8]?8'hff:row_next[7:0]):0;
                dropped<=closes-accepted;
                frame_reason<=reason_event & 5'b11110;
            end else begin
                frame_reason<=frame_reason|reason_event;
                if(I_valid) begin
                    row_total<=row_next[8]?8'hff:row_next[7:0];
                    if(row_next>row_peak) row_peak<=row_next[8]?8'hff:row_next[7:0];
                    dropped<=drop_next[16]?16'hffff:drop_next[15:0];
                end
            end
            if(sof) begin
                x<=0;y<=0;bank<=0;run_open<=0;run_seed<=0;seeded<=0;
                state<=IDLE;allocated<=1;pending<=0;row_scan<=0;row_floor<=0;match_y<=0;
                frame_limit<=enqueue_limit;frame_pending<=0;scanning<=0;
                phase<=0;outer<=1;eligible<=0;sort_step<=0;sort_pass<=0;span_ok<=0;stat_valid<=0;large_area<=0;
                if(frame_pending || scanning || state!=IDLE || queued!=0) O_limit<=1;
                for(j=0;j<3;j=j+1) row_count[j]<=0;
                for(j=0;j<=LABELS;j=j+1) begin
                    parent[j]<=j;if(SHARE_LAST_ROW==0)last_ref[j]<=0;ymax[j]<=0;if(!SEED_CACHE)seed_hits[j]<=0;
                if(STATS_DRAM==0)begin area[j]<=0;xmin[j]<={XB{1'b1}};xmax[j]<=0;ymin[j]<={YB{1'b1}};end
                end
            end else begin
                if(enqueue_limit) begin frame_limit<=1;O_limit<=1;end
                if(pop) begin
                    event_seed<=qseed_head;event_seed_count<=qseed_count;ex0<=qx0_head;ex1<=qx1_head;ey<=qy_head;eb<=qb_head;es<=qs_head;
                    // Materialize descriptors in FIFO order, using one write
                    // port rather than up to three input-side writes. All
                    // previous-row descriptors finish before current-row pop.
                    state<=MATCH;pending<=0;
                    // Runs arrive in increasing x. Discard only intervals
                    // ending before the next run; retain spanning intervals.
                    if(qy_head!=match_y) begin row_scan<=0;row_floor<=0;end
                    else row_scan<=row_floor;
                    match_y<=qy_head;
                end
                case(state)
                    MATCH:begin
                        if(ey==0 || row_scan>=row_count[pb]) state<=SELECT;
                        else if(row_read_x0>ex1) state<=SELECT;
                        else begin
                            if(overlap_root!=0) pending[overlap_root]<=1;
                            if(row_read_x1<ex0) row_floor<=row_scan+1'b1;
                            row_scan<=row_scan+1;
                        end
                    end
                    SELECT:begin
                        if(merge_root!=0) begin
                            root<=merge_root;pending[merge_root]<=0;state<=MERGE;
                        end else if(seed_small || (STRONG_START!=0 && !(MOTION_EDGE_GUARD!=0&&I_motion_only?event_seed_count!=0:event_seed))) begin
                            // Reject only a NEW short seed. Short runs already
                            // connected to a labelled body still contribute.
                            state<=IDLE;
                        end else if(allocated<=LABELS) begin
                            root<=allocated;allocated<=allocated+1;state<=ADD;
                        end else if(recycle_root!=0) begin
                            root<=recycle_root;parent[recycle_root]<=recycle_root;
                            if(!SEED_CACHE)begin seeded[recycle_root]<=0;seed_hits[recycle_root]<=0;end stat_valid[recycle_root]<=0;large_area[recycle_root]<=0;
                            if(STATS_DRAM==0)area[recycle_root]<=0;
                            span_ok[recycle_root]<=0;
                            if(STATS_DRAM==0)begin xmin[recycle_root]<={XB{1'b1}};xmax[recycle_root]<=0;
                                ymin[recycle_root]<={YB{1'b1}};end
                            ymax[recycle_root]<=0;
                            state<=ADD;
                        end else begin
                            root<=0;frame_limit<=1;O_limit<=1;state<=IDLE;
                        end
                    end
                    MERGE:begin
                        if(merge_root==0) state<=ADD;
                        else begin
                            pending[merge_root]<=0;
                            if(!SEED_CACHE)begin seeded[root]<=seeded[root]|seeded[merge_root];seeded[merge_root]<=0;
                            seed_hits[root]<=({1'b0,seed_hits[root]}+{1'b0,seed_hits[merge_root]})>31?5'd31:seed_hits[root]+seed_hits[merge_root];seed_hits[merge_root]<=0;end
                            for(j=1;j<=LABELS;j=j+1) if(parent[j]==merge_root) parent[j]<=root;
                            stat_valid[root]<=1;stat_valid[merge_root]<=0;
                            large_area[root]<=stat_new_area>=area_floor;large_area[merge_root]<=0;
                            if(STATS_DRAM==0)begin area[root]<=area_a+area_b;area[merge_root]<=0;end
                            span_ok[root]<=next_span_ok;
                            if(STATS_DRAM==0) if(xmin_b<xmin_a) xmin[root]<=xmin_b;
                            if(STATS_DRAM==0) if(xmax_b>xmax_a) xmax[root]<=xmax_b;
                            if(STATS_DRAM==0) if(ymin_b<ymin_a) ymin[root]<=ymin_b;
                            if(ymax_b>ymax_a) ymax[root]<=ymax_b;
                        end
                    end
                    ADD:begin
                        if(!SEED_CACHE)begin seeded[root]<=seeded[root]|event_seed;
                        seed_hits[root]<=({1'b0,seed_hits[root]}+{3'd0,event_seed_count})>31?5'd31:seed_hits[root]+event_seed_count;end
                        stat_valid[root]<=1;large_area[root]<=stat_new_area>=area_floor;
                        if(STATS_DRAM==0)area[root]<=area_a+(ex1-ex0+1);
                        span_ok[root]<=next_span_ok;
                        if(STATS_DRAM==0) if(ex0<xmin_a) xmin[root]<=ex0;
                        if(STATS_DRAM==0) if(ex1>xmax_a) xmax[root]<=ex1;
                        if(STATS_DRAM==0) if(ey<ymin_a) ymin[root]<=ey;
                        if(ey>ymax_a) ymax[root]<=ey;
                        state<=IDLE;
                        if(SHARE_LAST_ROW==0)last_ref[root]<=ey;
                    end
                endcase
                if(frame_pending && queued==0 && state==IDLE && !scanning) begin
                    scanning<=1;scan<=1;work_count<=0;work_limit<=frame_limit;
                    phase<=0;outer<=1;eligible<=0;sort_step<=0;sort_pass<=0;
                    best_score<=0;best_label<=0;best_x<=0;
                    object_total<=0;
                    wx0<=0;wx1<=0;wy0<=0;wy1<=0;
                end
                if(scanning) begin
                    if(phase==0) begin
                        if(scan<=LABELS) begin eligible[scan]<=candidate_valid;scan<=scan+1'b1;end
                        else begin phase<=1;scan<=1;outer<=1;end
                    end else if(phase==1) begin
                        if(scan>LABELS) begin phase<=2;scan<=1;end
                        else if(!eligible[scan] || outer>LABELS) begin scan<=scan+1'b1;outer<=1;end
                        else if(nested_small) begin eligible[scan]<=0;scan<=scan+1'b1;outer<=1;end
                        else outer<=outer+1'b1;
                    end else if(phase==2) begin
                        if(scan<=LABELS) begin
                            if(eligible[scan]) begin
                                if(work_count==0) begin
                                    object_total<=object_total+1'b1;
                                    if(object_total>=TARGETS) work_limit<=1;
                                end
                                if(best_label==0 || area_a>best_score ||
                                    (area_a==best_score && xmin_a<best_x)) begin
                                    best_score<=area_a;best_label<=scan[5:0];best_x<=xmin_a;
                                end
                            end
                            scan<=scan+1'b1;
                        end else if(best_label==0) begin phase<=3;sort_step<=0;sort_pass<=0;end
                        else begin scan<=best_label;phase<=5;end
                    end else if(phase==5) begin
                        // Selected statistics now share the normal stat_a port.
                        for(j=0;j<TARGETS;j=j+1)if(work_count==j)begin
                            wx0[j*16+:16]<=xmin_a;wx1[j*16+:16]<=xmax_a;
                            wy0[j*16+:16]<=ymin_a;wy1[j*16+:16]<=ymax_a;
                        end
                        eligible[scan]<=0;work_count<=work_count+1'b1;
                        best_score<=0;best_label<=0;best_x<=0;
                        if(work_count==TARGETS-1) begin phase<=3;sort_step<=0;sort_pass<=0;end
                        else begin phase<=2;scan<=1;end
                    end else if(phase==3) begin
                        // One adjacent compare/swap per clock for any configured K.
                        // Only final blanking work changes; streaming CCL is untouched.
                        if(sort_b<work_count &&
                           (wx0[sort_a*16+:16]>wx0[sort_b*16+:16] ||
                            (wx0[sort_a*16+:16]==wx0[sort_b*16+:16] && wy0[sort_a*16+:16]>wy0[sort_b*16+:16]))) begin
                            for(j=0;j<TARGETS;j=j+1)begin
                                if(sort_a==j)begin
                                    wx0[j*16+:16]<=wx0[sort_b*16+:16];wx1[j*16+:16]<=wx1[sort_b*16+:16];
                                    wy0[j*16+:16]<=wy0[sort_b*16+:16];wy1[j*16+:16]<=wy1[sort_b*16+:16];
                                end else if(sort_b==j)begin
                                    wx0[j*16+:16]<=wx0[sort_a*16+:16];wx1[j*16+:16]<=wx1[sort_a*16+:16];
                                    wy0[j*16+:16]<=wy0[sort_a*16+:16];wy1[j*16+:16]<=wy1[sort_a*16+:16];
                                end
                            end
                        end
                        if(work_count<2 || sort_pass==work_count-1) phase<=4;
                        else if(sort_step+2>=work_count-sort_pass) begin
                            sort_step<=0;sort_pass<=sort_pass+1'b1;
                        end else sort_step<=sort_step+1'b1;
                    end else begin
                        O_result_valid<=1;O_count<=work_count;O_limit<=work_limit;
                        O_row_peak<=row_peak;O_labels<=allocated-1'b1;
                        O_objects<=object_total;O_dropped<=dropped;O_frame_reason<=frame_reason;
                        O_xmin<=wx0;O_xmax<=wx1;O_ymin<=wy0;O_ymax<=wy1;
                        scanning<=0;frame_pending<=0;
                    end
                end
            end
            if(I_valid) begin
                run_open<=open_d;run_seed<=seed_d;run_start<=start_d;run_end<=end_d;
                row_count[cb]<=slots+accepted;
                if(O_pixel_x==IMG_WIDTH-4) begin
                    x<=0;y<=(O_pixel_y==IMG_HEIGHT-1)?0:O_pixel_y+1;
                    bank<=(O_pixel_y==IMG_HEIGHT-1 || cb==2)?0:cb+1;
                    if(O_pixel_y==IMG_HEIGHT-1) frame_pending<=1;
                end else begin x<=O_pixel_x+4;y<=O_pixel_y;end
            end
        end
    end
endmodule


// Parallel payload implementation; generic CCL defaults to legacy.
module rle_payload_parallel #(parameter XB=10)(
 input valid,sof,opened,short_run,last_word,
 input [XB-1:0] xpos,prior_start,prior_end,
 input [2:0] prior_seed,
 input [3:0] mask,seeds,
 output [3*XB-1:0] closed0,closed1,
 output [8:0] closed_count,
 output [2:0] closed_seed,
 output [2:0] count
);
 wire active_open=opened && !sof;
 wire [4:0] e;
 assign e[0]=valid && active_open && !mask[3] && (short_run || prior_start!=prior_end);
 assign e[1]=valid && mask[3] && !mask[2] && (short_run || active_open);
 assign e[2]=valid && mask[2] && !mask[1] && (short_run || mask[3]);
 assign e[3]=valid && mask[1] && !mask[0] && (short_run || mask[2]);
 assign e[4]=valid && last_word && mask[0] && (short_run || mask[1]);
 assign count={2'd0,e[0]}+{2'd0,e[1]}+{2'd0,e[2]}+{2'd0,e[3]}+{2'd0,e[4]};
 wire [XB-1:0] xs[0:4],xe[0:4];
 wire [2:0] sc[0:4];
 wire [3:0] prefix[1:4];
 assign xs[0]=prior_start;assign xe[0]=prior_end;assign sc[0]=prior_seed;
 assign xs[1]=active_open?prior_start:xpos;
 assign xs[2]=mask[3]?xs[1]:xpos+1'b1;
 assign xs[3]=mask[2]?xs[2]:xpos+2'd2;
 assign xs[4]=mask[1]?xs[3]:xpos+2'd3;
 assign prefix[1]=4'b1000;
 assign prefix[2]=mask[3]?4'b1100:4'b0100;
 assign prefix[3]=mask[2]?(mask[3]?4'b1110:4'b0110):4'b0010;
 assign prefix[4]=mask[1]?(mask[2]?(mask[3]?4'b1111:4'b0111):4'b0011):4'b0001;
 genvar j;
 generate for(j=1;j<=4;j=j+1)begin:g_record
  assign xe[j]=xpos+(j-1);
  wire [3:0] hits=seeds & prefix[j];
  wire [1:0] n0={1'b0,hits[0]}+{1'b0,hits[1]};
  wire [1:0] n1={1'b0,hits[2]}+{1'b0,hits[3]};
  wire [2:0] ns={1'b0,n0}+{1'b0,n1};
  wire [2:0] base=active_open && prefix[j][3]?prior_seed:3'd0;
  wire [3:0] total={1'b0,base}+{1'b0,ns};
  assign sc[j]=base>=4?base:(total>=4?3'd4:total[2:0]);
 end endgenerate
 wire [4:0] select0,select1,select2;
 assign select0={e[4]&&!(e[0]||e[1]||e[2]),e[3]&&!(e[0]||e[1]),e[2]&&!e[0],e[1],e[0]};
 assign select1={e[4]&&(e[0]||e[1]||e[2])&&!(e[0]&&e[2]),e[3]&&(e[0]||e[1]),e[2]&&e[0],2'b00};
 assign select2={e[4]&&e[0]&&e[2],4'b0000};
 wire [4:0] sel[0:2];
 assign sel[0]=select0;assign sel[1]=select1;assign sel[2]=select2;
 genvar s;
 generate for(s=0;s<3;s=s+1)begin:g_slot
  assign closed0[s*XB+:XB]=({XB{sel[s][0]}}&xs[0])|({XB{sel[s][1]}}&xs[1])|({XB{sel[s][2]}}&xs[2])|({XB{sel[s][3]}}&xs[3])|({XB{sel[s][4]}}&xs[4]);
  assign closed1[s*XB+:XB]=({XB{sel[s][0]}}&xe[0])|({XB{sel[s][1]}}&xe[1])|({XB{sel[s][2]}}&xe[2])|({XB{sel[s][3]}}&xe[3])|({XB{sel[s][4]}}&xe[4]);
  assign closed_count[s*3+:3]=({3{sel[s][0]}}&sc[0])|({3{sel[s][1]}}&sc[1])|({3{sel[s][2]}}&sc[2])|({3{sel[s][3]}}&sc[3])|({3{sel[s][4]}}&sc[4]);
  assign closed_seed[s]=closed_count[s*3+:3]>=4;
 end endgenerate
endmodule
