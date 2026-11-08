open_project camera_to_dsi_display.al
foreach {stage key expected} {
 place opt_timing high place ofv 70 place opt_max_density 50
 place target_density 0.6 place target_density_pp 0.7
 place aggressive_opt 20 place enhanced_beeline_opt 2 place enhanced_gp on
 place non_crit_opt on place post_place_opt high place enable_congestion_recovery on
 place seed 1 route post_route_opt high bitgen compress on
} {
 set value [get_param $stage $key -run phy_1]
 puts "CHECK $stage $key actual=$value expected=$expected"
 if {$value ne $expected} {error "GUI strategy mismatch: $stage $key"}
}
puts "PASS GUI physical strategy readback"
exit 0
