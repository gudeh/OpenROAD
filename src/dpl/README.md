# Detailed Placement

The detailed placement module in OpenROAD (`dpl`) is based on OpenDP, or 
Open-Source Detailed Placement Engine. Its key features are: 

-   Fence region.
-   Fragmented ROWs.

## Commands

```{note}
- Parameters in square brackets `[-param param]` are optional.
- Parameters without square brackets `-param2 param2` are required.
```

### Detailed Placement

The `detailed_placement` command performs detailed placement of instances
to legal locations after global placement.

```tcl
detailed_placement
    [-max_displacement disp|{disp_x disp_y}]
    [-disallow_one_site_gaps]
    [-report_file_name filename]
```

#### Options

| Switch Name | Description | 
| ----- | ----- |
| `-max_displacement` | Max distance that an instance can be moved (in microns) when finding a site where it can be placed. Either set one value for both directions or set `{disp_x disp_y}` for individual directions. The default values are `{0, 0}`, and the allowed values within are integers `[0, MAX_INT]`. |
| `-disallow_one_site_gaps` | Option is deprecated. |
| `-report_file_name` | File name for saving the report to (e.g. `report.json`.) |

### Set Placement Padding

The `set_placement_padding` command sets left and right padding in multiples
of the row site width. Use the `set_placement_padding` command before
legalizing placement to leave room for routing. Use the `-global` flag
for padding that applies to all instances. Use  `-instances`
for instance-specific padding.  The instances `insts` can be a list of instance
names, or an instance object returned by the SDC `get_cells` command. To
specify padding for all instances of a common master, use the `-filter`
"ref_name == <name>" option to `get_cells`.

```tcl
set_placement_padding   
    -global|-masters masters|-instances insts
    [-right site_count]
    [-left site_count]
```

#### Options

```{warning}
Either one of these flags must be set: `-global | -masters | -instances`.
The order of preference is `global > masters > instances`
```

| Switch Name | Description | 
| ----- | ----- |
| `-global` | Set padding globally using `left` and `right` values. |
| `-masters` |  Set padding only for these masters using `left` and `right` values. | 
| `-instances` | For `-instances`, you will set padding only for these insts using `left` and `right` values. |
| `-left` | Left padding (in site count). |
| `-right` | Right padding (in site count). |
| `instances` | Set padding for these list of instances. Not to be confused with the `-instances` switch above. |

### Filler Placement

The `filler_placement` command fills gaps between detail-placed instances
to connect the power and ground rails in the rows. `filler_masters` is a
list of master/macro names to use for filling the gaps. Wildcard matching
is supported, so `FILL*` will match, e.g., `FILLCELL_X1 FILLCELL_X16 FILLCELL_X2
FILLCELL_X32 FILLCELL_X4 FILLCELL_X8`.  To specify a different naming prefix
from `FILLER_` use `-prefix <new prefix>`.

```tcl 
filler_placement
    [-prefix prefix]
    [-verbose]
    filler_masters
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-prefix` | Prefix to name the filler cells. The default value is `FILLER_`. |
| `-verbose` | Print the filler cell usage. |
| `filler_masters` | Filler master cells. | 

### Remove Fillers

This command removes all filler cells.

```tcl
remove_fillers 
```

### Check Placement

The `check_placement` command checks the placement legality. It returns
`0` if the placement is legal.

```tcl
check_placement
    [-verbose]
    [-disallow_one_site_gaps]
    [-report_file_name filename]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-verbose` | Enable verbose logging. |
| `-disallow_one_site_gaps` | Option is deprecated. |
| `-report_file_name` | File name for saving the report to (e.g. `report.json`.) |

### Optimize Mirroring

The `optimize_mirroring` command mirrors instances about the Y axis in
a weak attempt to reduce the total half-perimeter wirelength (HPWL).

```tcl
optimize_mirroring
```
### Improve Placement

The `improve_placement` command optimizes a given placed design.

```tcl
improve_placement
    [-random_seed seed]
    [-max_displacement disp|{disp_x disp_y}]
    [-disallow_one_site_gaps]
```

### Set Cells File

The `set_cells_file` command specifies a JSON file containing cell information that is used by the `swap_cells_anneal` and `optimize_pin_placement` commands. This file defines:
- Multibit cell groups for cell swapping during annealing
- Pin swap and pin permutation information for pin placement optimization

```tcl
set_cells_file cells_file
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `cells_file` | Path to the JSON file containing cell information. |

### Swap Cells Anneal

The `swap_cells_anneal` command uses simulated annealing optimization to upsize cells by merging them into larger multibit cells. This process helps optimize the design by combining compatible cells into more efficient multibit configurations while maintaining placement legality.

```tcl
swap_cells_anneal
    [-max_iterations iterations]
    [-initial_temperature temperature]
    [-alpha alpha_value]
    [-seed seed]
    [-max_displacement disp|{disp_x disp_y}]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-max_iterations` | Maximum number of iterations for annealing (default: 1000). |
| `-initial_temperature` | Initial temperature for annealing (default: 100.0). Higher values allow more unfavorable merges initially. |
| `-alpha` | Cooling rate for temperature (default: 0.95). Controls how quickly the algorithm becomes more selective about accepting merges. |
| `-seed` | Random seed for reproducibility (default: 1). |
| `-max_displacement` | Maximum displacement allowed when repositioning merged cells, either single value for both directions or {disp_x disp_y}. |

The command requires cell group definitions to be provided through the `set_cells_file` command before execution.

### Optimize Pin Placement

The `optimize_pin_placement` command performs three types of pin optimizations to improve the design's wire length:

1. **Multibit Pin Optimization**: Rearranges the bit ordering within multibit cells.
2. **Pin Swaps**: Performs pin swaps based on predefined swappable pin groups within each cell.
3. **Pin Permutations**: Applies more complex pin rearrangements using predefined permutation groups.

The command iteratively applies these optimizations to each standard cell until no further improvements can be made.

```tcl
optimize_pin_placement
```

This command uses pin swap and permutation information provided through the `set_cells_file` command.

### Set Phi Cut Cell

The `set_phi_cut_cell` command specifies a cell to be used as a phi-cut cell.

```tcl
set_phi_cut_cell cell_name
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `cell_name` | Name of the cell to be used as phi-cut cell. |

### Place Cut Phi Cells

The `place_phi_cut_cells` command places the specified phi-cut cells.

```tcl
place_phi_cut_cells
```

### Set Tap Phi Cell

The `set_tap_phi_cell` command specifies a cell to be used as a tap phi cell.

```tcl
set_tap_phi_cell cell_name
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `cell_name` | Name of the cell to be used as tap phi cell. |

### Disallow Odd Sites

```{warning}
This command is deprecated and will be removed in a future release.
```

The `disallow_odd_sites` command disallows placement in odd-numbered sites.

```tcl
disallow_odd_sites
```

## Useful Developer Commands

If you are a developer, you might find these useful. More details can be found in the [source file](./src/Opendp.cpp) or the [swig file](./src/Opendp.i).

| Command Name | Description |
| ----- | ----- |
| `detailed_placement_debug` | Debug detailed placement. |
| `get_masters_arg` | Get masters from a design. |
| `get_inst_bbox` | Get bounding box of an instance. |
| `get_inst_grid_bbox` | Get grid bounding box of an instance. |
| `format_grid` | Format grid (takes in length `x` and site width `w` as inputs). |
| `get_row_site` | Get row site name.

## Example scripts

Examples scripts demonstrating how to run `dpl` on a sample design of `aes` as follows:

```shell
./test/aes.tcl
```

## Regression tests

There are a set of regression tests in `./test`. Refer to this [section](../../README.md#regression-tests) for more information.

Simply run the following script: 

```shell
./test/regression
```

## Limitations

## FAQs

Check out [GitHub discussion](https://github.com/The-OpenROAD-Project/OpenROAD/discussions/categories/q-a?discussions_q=category%3AQ%26A+opendp+in%3Atitle)
about this tool.

## Authors

-   SangGi Do and Mingyu Woo (respective Ph. D. advisors: Seokhyeong Kang,
    Andrew B. Kahng).
-   Rewrite and port to OpenDB/OpenROAD by James Cherry, Parallax Software

## References
1. Do, S., Woo, M., & Kang, S. (2019, May). Fence-region-aware mixed-height standard cell legalization. In Proceedings of the 2019 on Great Lakes Symposium on VLSI (pp. 259-262). [(.pdf)](https://dl.acm.org/doi/10.1145/3299874.3318012)

## License

BSD 3-Clause License. See [LICENSE](LICENSE) file.
