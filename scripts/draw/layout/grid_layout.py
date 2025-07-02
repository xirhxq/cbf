import math
from matplotlib.gridspec import GridSpec
from components.components import *


class GridLayout:
    def __init__(self, fig, plot_list, expand=True, n=None, robot_id=None, **kwargs):
        self.fig = fig
        self.plot_list = plot_list

        self.expand = expand
        self.n = n
        self.robot_id = robot_id


        self.layout_config = self._get_layout()


    def _get_layout(self):
        side_list = [s for s in self.plot_list if s != 'map']
        side_num = len(side_list)

        layout_config = {}
        if side_num == 0:
            layout_config['rows'] = 1
            layout_config['cols'] = 1
            layout_config['components'] = [
                {
                    'name': 'map',
                    'grid': [[None, None], [None, None]],
                    **REGISTRIED_COMPONENTS['map']
                }
            ]
        else:
            side_cols = min(4, self.n)
            side_rows = math.ceil(self.n / side_cols)

            layout_config = {
                'components': []
            }

            if 'map' in self.plot_list:
                map_cols = math.ceil(side_cols / 2)
                layout_config['components'].append(
                    {
                        'grid': [[None, None], [None, map_cols]],
                        **REGISTRIED_COMPONENTS['map']
                    }
                )
            else:
                map_cols = 0

            layout_config['rows'] = side_rows * side_num
            layout_config['cols'] = side_cols + map_cols

            def parse_group_layout(layout, name, grid, n):
                i = grid[0][0] if grid[0][0] is not None else 0
                j = grid[1][0] if grid[1][0] is not None else 0

                if i >= grid[0][1] or j >= grid[1][1]:
                    raise ValueError(
                        f'Grid exceeds: ({i}, {j}) is outside the range of '
                        f'({grid[0][0]}, {grid[1][0]}) to ({grid[0][1]}, {grid[1][1]})'
                    )

                for id in range(n):
                    layout.append(
                        {
                            'grid': [i, j],
                            'robot_id': id,
                            **REGISTRIED_COMPONENTS[name]
                        }
                    )

                    j = j + 1
                    if j >= grid[1][1]:
                        j = grid[1][0]
                        i = i + 1

            for index, item in enumerate(side_list):
                if self.expand:
                    parse_group_layout(
                        layout_config['components'],
                        item,
                        [
                            [index * side_rows, (index + 1) * side_rows],
                            [map_cols, layout_config['cols']]
                        ],
                        self.n
                    )
                else:
                    layout_config['components'].append(
                        {
                            'grid': [
                                [index * side_rows, (index + 1) * side_rows],
                                [map_cols, layout_config['cols']]
                            ],
                            'robot_id': self.robot_id,
                            **REGISTRIED_COMPONENTS[item]
                        }
                    )

        return layout_config

    def allocate_axes(self):
        rows = self.layout_config.get("rows", 2)
        cols = self.layout_config.get("cols", 2)
        gs = GridSpec(rows, cols)
        axes_map = []
        for comp_cfg in self.layout_config.get("components", []):
            grid = comp_cfg.get("grid", [0, 0])
            row_spec, col_spec = self._get_grid(grid)
            comp_cfg["ax"] = self.fig.add_subplot(gs[row_spec, col_spec])
            axes_map.append(comp_cfg)
        return axes_map

    def _get_grid(self, grid):
        if isinstance(grid, list) and all(isinstance(g, list) for g in grid):
            row_slice = self._parse_slice(grid[0])
            col_slice = self._parse_slice(grid[1])
            return row_slice, col_slice
        elif isinstance(grid, list) and len(grid) == 2:
            row, col = grid
            return row, col
        else:
            raise ValueError(f"Invalid grid format: {grid}")

    def _parse_slice(self, slice_list):
        if not (isinstance(slice_list, list) and len(slice_list) <= 2):
            raise ValueError("Slice must be a list of 1 or 2 elements.")

        start = slice_list[0] if slice_list[0] is not None else None
        stop = slice_list[1] if len(slice_list) > 1 and slice_list[1] is not None else None

        return slice(start, stop)
