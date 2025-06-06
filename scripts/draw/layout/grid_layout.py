from matplotlib.gridspec import GridSpec


class GridLayout:
    def __init__(self, fig, layout_config):
        self.fig = fig
        self.layout_config = layout_config
        rows = layout_config.get("rows", 2)
        cols = layout_config.get("cols", 2)
        self.gs = GridSpec(rows, cols)

    def allocate_axes(self):
        axes_map = {}
        for comp_cfg in self.layout_config.get("components", []):
            name = comp_cfg["name"]
            grid = comp_cfg.get("grid", [0, 0])

            if isinstance(grid, list) and all(isinstance(g, list) for g in grid):
                row_slice = self._parse_slice(grid[0])
                col_slice = self._parse_slice(grid[1])
                axes_map[name] = self.fig.add_subplot(self.gs[row_slice, col_slice])
            elif isinstance(grid, list) and len(grid) == 2:
                row, col = grid
                axes_map[name] = self.fig.add_subplot(self.gs[row, col])
            else:
                raise ValueError(f"Invalid grid format: {grid}")
        return axes_map

    def _parse_slice(self, slice_list):
        if not (isinstance(slice_list, list) and len(slice_list) <= 2):
            raise ValueError("Slice must be a list of 1 or 2 elements.")

        start = slice_list[0] if slice_list[0] is not None else None
        stop = slice_list[1] if len(slice_list) > 1 and slice_list[1] is not None else None

        return slice(start, stop)
