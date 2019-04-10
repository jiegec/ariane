set partNumber $::env(XILINX_PART)
set boardName  $::env(XILINX_BOARD)

set ipName xlnx_processing_system7

create_project $ipName . -force -part $partNumber
set_property board_part $boardName [current_project]

create_ip -name processing_system7 -vendor xilinx.com -library ip -module_name $ipName
set_property -dict [list CONFIG.PCW_FPGA_FCLK0_ENABLE {1} \
    CONFIG.PCW_FPGA_FCLK1_ENABLE {0} \
    CONFIG.PCW_FPGA_FCLK2_ENABLE {0} \
    CONFIG.PCW_FPGA_FCLK3_ENABLE {0} \
    CONFIG.PCW_USE_M_AXI_GP0 {0} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_CLK0_FREQ {200000000} \
    CONFIG.PCW_FCLK0_PERIPHERAL_DIVISOR0 {3} \
    CONFIG.PCW_FCLK0_PERIPHERAL_DIVISOR1 {3} \
] [get_ips $ipName]

generate_target {instantiation_template} [get_files ./$ipName.srcs/sources_1/ip/$ipName/$ipName.xci]
generate_target all [get_files  ./$ipName.srcs/sources_1/ip/$ipName/$ipName.xci]
create_ip_run [get_files -of_objects [get_fileset sources_1] ./$ipName.srcs/sources_1/ip/$ipName/$ipName.xci]
launch_run -jobs 8 ${ipName}_synth_1
wait_on_run ${ipName}_synth_1
