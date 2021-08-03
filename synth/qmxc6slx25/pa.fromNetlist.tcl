
# PlanAhead Launch Script for Post-Synthesis floorplanning, created by Project Navigator

create_project -name qmxc6slx25 -dir "D:/Users/benit/Downloads/UnAmiga/ZXDos/Cores/msx1fpga-1.3/synth/qmxc6slx25/planAhead_run_2" -part xc6slx25ftg256-2
set_property design_mode GateLvl [get_property srcset [current_run -impl]]
set_property edif_top_file "D:/Users/benit/Downloads/UnAmiga/ZXDos/Cores/msx1fpga-1.3/synth/qmxc6slx25/qmxc6slx25_top.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {D:/Users/benit/Downloads/UnAmiga/ZXDos/Cores/msx1fpga-1.3/synth/qmxc6slx25} {ipcore_dir} }
set_property target_constrs_file "D:/Users/benit/Downloads/UnAmiga/ZXDos/Cores/msx1fpga-1.3/src/syn-qmxc6slx25/qmxc6slx25_pins.ucf" [current_fileset -constrset]
add_files [list {D:/Users/benit/Downloads/UnAmiga/ZXDos/Cores/msx1fpga-1.3/src/syn-qmxc6slx25/qmxc6slx25_pins.ucf}] -fileset [get_property constrset [current_run]]
link_design
