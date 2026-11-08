`timescale 1ns/1ps
// Exact dual-port history semantics of the camera reference IP, with depth
// sized to ceil_pow2(image_width/4), not fixed 2048. Original read/write
// modes and one-clock read are retained. No state or pixel latency added.
module line_history_ram #(parameter WIDTH=33,ADDR_BITS=9)(
 input clka,wea,input [ADDR_BITS-1:0] addra,input [WIDTH-1:0] dia,
 input clkb,input [ADDR_BITS-1:0] addrb,output [WIDTH-1:0] dob
);
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(WIDTH),.DATA_WIDTH_B(WIDTH),
 .ADDR_WIDTH_A(ADDR_BITS),.ADDR_WIDTH_B(ADDR_BITS),
 .DATA_DEPTH_A(1<<ADDR_BITS),.DATA_DEPTH_B(1<<ADDR_BITS),
 .MODE("DP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
 .WRITEMODE_A("NORMAL"),.WRITEMODE_B("READBEFOREWRITE"),
 .RESETMODE_A("ASYNC"),.RESETMODE_B("ASYNC"),
 .OREGSET_A("RESET"),.OREGSET_B("RESET"),.SSROVERCE("DISABLE"),
 .ECC_ENCODE("DISABLE"),.ECC_DECODE("DISABLE"),
 .ASYNC_RESET_RELEASE_A("ASYNC"),.ASYNC_RESET_RELEASE_B("ASYNC"),
 .CLKMODE("ASYNC"),.INIT_FILE("NONE"),.FILL_ALL("NONE"),.IMPLEMENT("20K(FAST)")) memory(
 .doa(),.dia(dia),.addra(addra),.cea(1'b1),.ocea(1'b0),.clka(clka),.wea(wea),.rsta(1'b0),.bea(1'b0),
 .dob(dob),.dib({WIDTH{1'b0}}),.addrb(addrb),.ceb(1'b1),.oceb(1'b0),.clkb(clkb),.web(1'b0),.rstb(1'b0),.beb(1'b0),
 .ecc_sbiterr(),.ecc_dbiterr(),.ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0));
endmodule
