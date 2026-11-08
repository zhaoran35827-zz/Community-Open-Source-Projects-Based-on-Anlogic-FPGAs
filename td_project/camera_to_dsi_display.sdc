create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 4.167 -waveform {0.000 2.083} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]

rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]


set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] \
    -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

# Asynchronous mechanical contacts, synchronized in the video domain.
set_false_path -from [get_ports {I_switch[*]}] -to [get_regs {u_switch_control/sync_meta*}]

# External asynchronous contact; only the first synchronizer is excepted.
set_false_path -from [get_ports {I_traffic_switch_n}] -to [get_regs {u_traffic_switch/sync_meta}]

# Bundled-data mailbox between 50 MHz and HDMI: only the two FIRST toggle
# synchronizers intentionally accept arbitrary phase. Second stages and the
# stable held payload remain timed; do not cut the whole clock domains.
set_false_path -from [get_regs {u_traffic_status/acknowledge}] -to [get_regs {u_traffic_status/ack_meta}]
set_false_path -from [get_regs {u_traffic_status/request}] -to [get_regs {u_traffic_status/req_meta}]

# The held
# mailbox payload is intentionally asynchronous at ONLY its first data stage.
# It is bounded to < one destination cycle and gets two data stages plus a
# settling cycle before use/ack. Do not time it as a same-edge synchronous bus.
set_max_delay -datapath_only 10.000 -from [get_regs {u_traffic_status/held*}] -to [get_regs {u_traffic_status/payload_meta*}]
set_false_path -hold -from [get_regs {u_traffic_status/held*}] -to [get_regs {u_traffic_status/payload_meta*}]

# PIPE observes a sticky24MHz-domain status through TWO75MHz registers.
# Only its FIRST sampling stage may accept arbitrary source phase. Keep the
# second stage and all functional video paths timed.
set_false_path -from [get_regs {S_dbg_cam_cfg_seen}] -to [get_regs {S_dbg_cam_cfg_seen_sync[0]}]

# External gimbal keys: only asynchronous contacts to FIRST sampling stage.
set_false_path -from [get_ports {I_gimbal_key_n[*]}] -to [get_regs {u_manual_gimbal/key_meta*}]
