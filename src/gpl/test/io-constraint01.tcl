source helpers.tcl
set test_name io-constraint01
read_liberty ./library/nangate45/NangateOpenCellLibrary_typical.lib

read_lef ./nangate45.lef
read_def ./$test_name.def

create_clock -name core_clock -period 2 clk

set_wire_rc -signal -layer metal3
set_wire_rc -clock -layer metal5

#set_io_pin_constraint -direction input -region left:*
# set_io_pin_constraint -direction output -region right:*

#outputs
set_io_pin_constraint -pin_names {resp_msg[3] resp_msg[4] resp_msg[5] resp_msg[6] resp_msg[7] resp_msg[8] resp_msg[9]} -region bottom:10-28
set_io_pin_constraint -pin_names {resp_msg[2] resp_msg[1] resp_msg[15] resp_msg[14] resp_msg[13] resp_msg[12] resp_msg[11]} -region top:0-10
set_io_pin_constraint -pin_names {resp_msg[10] resp_msg[0]} -region top:25-30

#inputs
set_io_pin_constraint -pin_names {req_msg[9] req_msg[8] req_msg[7] req_msg[6] req_msg[5] req_msg[4] req_msg[3] req_msg[31]} -region left:10-20
set_io_pin_constraint -pin_names {req_msg[30] req_msg[2] req_msg[29] req_msg[28] req_msg[27] req_msg[26] req_msg[25] req_msg[24]} -region right:15-25

#from def file
#unconstrained and unplaced: req_msg --> 23, 22, 21, 20, 1, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10
#placed: req_msg[0], resp_val, resp_rdy, reset, req_val, req_rdy, clk

global_placement_debug -initial
set_debug_level GPL callbacks 3
set_debug_level GPL io_constraint 2
global_placement -timing_driven -timing_driven_net_reweight_overflow [list 80 70 60 50 40 30 20]

# check reported wns
estimate_parasitics -placement
report_worst_slack

set def_file [make_result_file $test_name.def]
write_def $def_file
diff_file $def_file $test_name.defok
