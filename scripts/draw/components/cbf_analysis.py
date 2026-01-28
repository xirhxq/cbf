from utils import *
from .base import BaseComponent
import matplotlib.pyplot as plt


class ConstraintCBFsComponent(BaseComponent):
    """Plot constraint CBFs (must satisfy h >= 0) for a specific robot"""
    def __init__(self, ax, data, robot_id, **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.robot_name = f"UAV #{robot_id + 1}"

        # First pass: discover all constraint CBF names for this robot
        self.cbf_names = set()
        for frame in self.data:
            robot_data = frame["robots"][robot_id]
            cbf_no_slack = robot_data.get("cbfNoSlack", {})
            if cbf_no_slack:
                self.cbf_names.update(cbf_no_slack.keys())

        # Remove duplicate (empty) entries
        self.cbf_names.discard('')
        self.cbf_names = sorted(list(self.cbf_names))

        # Second pass: extract time series data for each CBF
        self.time = []
        self.cbf_values = {name: [] for name in self.cbf_names}

        for frame in self.data:
            self.time.append(frame["runtime"])
            robot_data = frame["robots"][robot_id]
            cbf_no_slack = robot_data.get("cbfNoSlack", {})

            for name in self.cbf_names:
                self.cbf_values[name].append(cbf_no_slack.get(name, np.nan))

        self._plot()

    def _plot(self):
        # Define colors for different CBF types
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

        for idx, cbf_name in enumerate(self.cbf_names):
            # Create clean LaTeX-style label
            if 'fixedCommCBF' in cbf_name:
                # Extract anchor ID, e.g., "fixedCommCBF(#1)" -> "h_loc^1"
                anchor_part = cbf_name.split("(")[1].split(")")[0] if "(" in cbf_name else ""
                # Remove '#' prefix if present and convert to clean label
                if anchor_part.startswith('#'):
                    anchor_num = anchor_part[1:]  # Remove '#'
                    label = f'$h_{{loc}}^{{{anchor_num}}}$ (UAV {anchor_num})'
                elif 'base' in anchor_part:
                    base_num = anchor_part.replace('base-', '')
                    label = f'$h_{{loc}}^{{(base {base_num})}}$'
                else:
                    label = f'$h_{{loc}}^{{{anchor_part}}}$'
            elif 'safetyCBF' in cbf_name:
                label = '$h_{safety}$'
            else:
                # Fallback: use simple text label
                label = cbf_name

            color = colors[idx % len(colors)]
            self.ax.plot(self.time, self.cbf_values[cbf_name], label=label, linewidth=1.5, color=color)

        self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('CBF Value')
        self.ax.set_title(f'Constraint CBFs - {self.robot_name}')
        self.ax.legend(loc='best')
        self.ax.grid(True, alpha=0.3)


class TaskCBFsComponent(BaseComponent):
    """Plot task CBFs (h >= -delta) for a specific robot"""
    def __init__(self, ax, data, robot_id, **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.robot_name = f"UAV #{robot_id + 1}"

        # Extract time and CBF values
        self.time = []
        self.cvt = []
        self.yaw = []

        for frame in self.data:
            self.time.append(frame["runtime"])
            robot_data = frame["robots"][robot_id]

            # Get task CBFs
            cbf_slack = robot_data.get("cbfSlack", {})
            if cbf_slack:
                self.cvt.append(cbf_slack.get("cvtCBF", np.nan))
                self.yaw.append(cbf_slack.get("cvtYawCBF", np.nan))
            else:
                self.cvt.append(np.nan)
                self.yaw.append(np.nan)

        self._plot()

    def _plot(self):
        # Create twin axis for yaw CBF (very small values)
        ax2 = self.ax.twinx()

        # Plot CVT CBF on left axis
        line1, = self.ax.plot(self.time, self.cvt, label='$h_{search}$', linewidth=1.5, color='blue')
        self.ax.set_ylabel('CVT CBF Value', color='blue')
        self.ax.tick_params(axis='y', labelcolor='blue')

        # Plot Yaw CBF on right axis
        line2, = ax2.plot(self.time, self.yaw, label='$h_{yaw}$', linewidth=1.5, color='orange')
        ax2.set_ylabel('Yaw CBF Value', color='orange')
        ax2.tick_params(axis='y', labelcolor='orange')

        self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)
        ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)

        self.ax.set_xlabel('Time (s)')
        self.ax.set_title(f'Task CBFs - {self.robot_name}')

        # Combined legend
        lines = [line1, line2]
        labels = [line.get_label() for line in lines]
        self.ax.legend(lines, labels, loc='best')

        self.ax.grid(True, alpha=0.3)
        ax2.grid(False)


class SlackVarsComponent(BaseComponent):
    """Plot slack variables for task CBFs"""
    def __init__(self, ax, data, robot_id, **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.robot_name = f"UAV #{robot_id + 1}"

        # Extract time and slack values
        self.time = []
        self.delta_cvt = []
        self.delta_yaw = []

        # First, discover the order of task CBFs
        task_cbf_names = []
        for frame in self.data:
            robot_data = frame["robots"][robot_id]
            opt = robot_data.get("opt", {})
            cbf_slack = opt.get("cbfSlack", [])
            if len(cbf_slack) > 0:
                task_cbf_names = [cbf.get("name") for cbf in cbf_slack]
                break

        # Now extract slack values in the same order
        for frame in self.data:
            self.time.append(frame["runtime"])
            robot_data = frame["robots"][robot_id]
            opt = robot_data.get("opt", {})
            slacks = opt.get("slacks", [])

            # Map slacks array to CBFs by index
            slack_dict = {}
            for idx, cbf_name in enumerate(task_cbf_names):
                if idx < len(slacks):
                    slack_dict[cbf_name] = slacks[idx]

            self.delta_cvt.append(slack_dict.get("cvtCBF", np.nan))
            self.delta_yaw.append(slack_dict.get("cvtYawCBF", np.nan))

        self._plot()

    def _plot(self):
        # Check if we need dual axis due to scale differences
        cvt_max = max([v for v in self.delta_cvt if not np.isnan(v)], default=0)
        yaw_max = max([v for v in self.delta_yaw if not np.isnan(v)], default=0)
        use_dual_axis = abs(cvt_max) / abs(yaw_max) > 100 if yaw_max != 0 else False

        if use_dual_axis:
            # Create twin axis for yaw slack (very different scale)
            ax2 = self.ax.twinx()

            # Plot CVT slack on left axis
            line1, = self.ax.plot(self.time, self.delta_cvt, label='$\\delta_{search}$', linewidth=1.5, color='blue')
            self.ax.set_ylabel('Search Slack Value', color='blue')
            self.ax.tick_params(axis='y', labelcolor='blue')

            # Plot Yaw slack on right axis
            line2, = ax2.plot(self.time, self.delta_yaw, label='$\\delta_{yaw}$', linewidth=1.5, color='orange')
            ax2.set_ylabel('Yaw Slack Value', color='orange')
            ax2.tick_params(axis='y', labelcolor='orange')

            self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)
            ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)

            # Combined legend
            lines = [line1, line2]
            labels = [line.get_label() for line in lines]
            self.ax.legend(lines, labels, loc='best')

            self.ax.grid(True, alpha=0.3)
            ax2.grid(False)
        else:
            # Single axis for both
            if any(not np.isnan(v) for v in self.delta_cvt):
                self.ax.plot(self.time, self.delta_cvt, label='$\\delta_{search}$', linewidth=1.5, color='blue')

            if any(not np.isnan(v) for v in self.delta_yaw):
                self.ax.plot(self.time, self.delta_yaw, label='$\\delta_{yaw}$', linewidth=1.5, color='orange')

            self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)
            self.ax.legend(loc='best')
            self.ax.grid(True, alpha=0.3)

        self.ax.set_xlabel('Time (s)')
        self.ax.set_title(f'Slack Variables - {self.robot_name}')
