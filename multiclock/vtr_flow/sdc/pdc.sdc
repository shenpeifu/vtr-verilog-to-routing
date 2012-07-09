create_clock -period 0 -name virtual_clock
set_input_delay -clock virtual_clock -max 0 [get_ports{*}]
set_output_delay -clock virtual_clock -max 0 [get_ports{*}]

