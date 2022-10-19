
#
# Create Clocks
#
set overconstraint 0.20

create_clock -period 8 -name clk_i [get_ports {clk_i}]
create_generated_clock -divide_by 1 -source [get_ports {clk_i}] -name clk100 [get_pins {clkdiv_inst/CLKOP}]
create_generated_clock -divide_by 2 -source [get_ports {clk_i}] -name clk50  [get_pins {clkdiv_inst/CLKOS}]
create_generated_clock -divide_by 4 -source [get_ports {clk_i}] -name clk25  [get_pins {clkdiv_inst/CLKOS2}]
create_generated_clock -divide_by 5 -source [get_ports {clk_i}] -name clk20  [get_pins {clkdiv_inst/CLKOS3}]

#This breaks functionality in HW
create_clock -period 50.0 -name cpu_jtag_tck [get_ports {dbg_tck}]

set_input_delay 25 -clock [get_clocks {dbg_tck}] [get_ports {dbg_tms}]
set_input_delay 25 -clock [get_clocks {dbg_tck}] [get_ports {dbg_tdi}]

set_output_delay 25 -clock [get_clocks {dbg_tck}] [get_ports {dbg_tdo}]

set_clock_groups -asynchronous -group {clk100 clk50 clk25 clk20} -group {dbg_tck}


#
# Multi-cycle paths are present in AHB2APB bridge
# Is APB bus multi-cycle?
#
