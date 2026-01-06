source "helpers.tcl"

read_def simple01.def

set_placement_padding -global -left 2 -right 2

# Set debug instance to start logging
detailed_placement_debug -instance _277_

detailed_placement -max_displacement {2 3}
