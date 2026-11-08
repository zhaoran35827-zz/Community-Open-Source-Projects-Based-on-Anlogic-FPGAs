module data_96bit_to_128bit (
 input wire I_clk,I_rst_n,I_96b_frame_start,I_96b_valid,
 input wire [95:0] I_96b_data,
 output wire O_128b_frame_start,O_128b_valid,
 output wire [127:0] O_128b_data
);
 rgb96_to128_stream u_stream(
 .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(I_96b_frame_start),
 .I_valid(I_96b_valid),.I_data(I_96b_data),
 .O_start(O_128b_frame_start),.O_valid(O_128b_valid),.O_data(O_128b_data));
endmodule
