set_property PACKAGE_PIN C2 [get_ports reset]
set_property IOSTANDARD LVCMOS33 [get_ports reset]
set_property PACKAGE_PIN D9 [get_ports reset_rtl]
set_property IOSTANDARD LVCMOS33 [get_ports reset_rtl]

## Clock signal
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { sys_clock }]; 
#create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { sys_clock }];

#set_property -dict { PACKAGE_PIN G18    IOSTANDARD LVCMOS33 } [get_ports { eth_ref_clk }]; 

