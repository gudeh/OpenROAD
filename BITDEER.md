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
