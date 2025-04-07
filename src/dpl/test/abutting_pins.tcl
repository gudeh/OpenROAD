source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_lef fake_cell_long_pins.lef
read_def abutting_pins.def
detailed_placement
filler_placement FILL*
check_placement

set def_file [make_result_file abutting_pins.def]
write_def $def_file
diff_file $def_file abutting_pins.defok
