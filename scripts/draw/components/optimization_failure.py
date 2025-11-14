from utils import *
from .base import BaseComponent
import matplotlib.pyplot as plt
import matplotlib.lines as mlines


def optimization_failure_interpreter(data):
    """
    Extract optimization failure data from robots
    Returns processed data for visualization showing failure flags over time
    """
    processed_data = {}
    runtime = [frame["runtime"] for frame in data["state"]]

    # Get all robot IDs
    if data["state"]:
        robot_ids = [robot["id"] for robot in data["state"][0]["robots"]]
        num_robots = len(robot_ids)
    else:
        return processed_data

    # Initialize failure tracking for each robot
    for robot_id in robot_ids:
        processed_data[robot_id] = {
            'runtime': runtime,
            'failures': [],
            'failure_types': []
        }

    # Extract failure information for each robot over time
    for frame in data["state"]:
        for robot in frame["robots"]:
            robot_id = robot["id"]
            opt_data = robot.get("opt", {})

            # Check if optimization failed
            status = opt_data.get("status", "success")
            solver_info = opt_data.get("solver_info", {})
            solver_status = solver_info.get("status", "")

            # Mark as failure if status is failed or solver status indicates failure
            is_failure = (status == "failed" or
                         solver_status in ["infeasible", "unbounded", "inf_or_unbounded", "error"])

            failure_type = "success"
            if is_failure:
                failure_type = solver_status if solver_status in ["infeasible", "unbounded", "inf_or_unbounded"] else "failed"

            processed_data[robot_id]['failures'].append(1 if is_failure else 0)
            processed_data[robot_id]['failure_types'].append(failure_type)

    return processed_data


class OptimizationFailureComponent(BaseComponent):
    """
    Component for visualizing optimization failure patterns
    Shows when each robot's optimization fails over time
    """

    def __init__(self, fig, data, **kwargs):
        self.fig = fig
        self.data = data

        self.title = kwargs.get('title', 'Optimization Failure Timeline')
        self.xlabel = kwargs.get('xlabel', 'Time / s')
        self.ylabel = kwargs.get('ylabel', 'Robot ID')

        self.show_legend = kwargs.get('show_legend', True)
        self.time_range = kwargs.get('time_range', None)

        # Visual style
        self.success_color = kwargs.get('success_color', 'lightgreen')
        self.failure_color = kwargs.get('failure_color', 'red')
        self.infeasible_color = kwargs.get('infeasible_color', 'orange')
        self.unbounded_color = kwargs.get('unbounded_color', 'purple')

        # Process data
        self.processed_data = optimization_failure_interpreter(data)

        if not self.processed_data:
            raise ValueError("No optimization data found for failure visualization")

        self._apply_time_filter()
        self._create_plot()

    def _apply_time_filter(self):
        if not self.time_range or len(self.time_range) != 2:
            return

        start_time, end_time = self.time_range

        for robot_id, robot_data in self.processed_data.items():
            runtime = robot_data['runtime']
            if len(runtime) < 2:
                continue

            start_idx = np.searchsorted(runtime, start_time)
            end_idx = np.searchsorted(runtime, end_time, side='right')

            if end_idx > start_idx:
                self.processed_data[robot_id]['runtime'] = runtime[start_idx:end_idx]
                self.processed_data[robot_id]['failures'] = robot_data['failures'][start_idx:end_idx]
                self.processed_data[robot_id]['failure_types'] = robot_data['failure_types'][start_idx:end_idx]

    def _create_plot(self):
        """Create the failure timeline plot with discrete time points"""
        if not self.processed_data:
            return

        robot_ids = sorted(self.processed_data.keys())
        num_robots = len(robot_ids)

        # Clear existing axes created by GridLayout
        for ax in self.fig.axes[:]:
            ax.remove()

        ax = self.fig.add_subplot(1, 1, 1)

        # Plot each robot's failures as discrete points
        for i, robot_id in enumerate(robot_ids):
            robot_data = self.processed_data[robot_id]
            runtime = robot_data['runtime']
            failures = robot_data['failures']
            failure_types = robot_data['failure_types']

            # Separate successes and different types of failures
            success_times = []
            infeasible_times = []
            unbounded_times = []
            other_error_times = []

            # Process all time points - both successes and failures
            for j in range(len(runtime)):
                # Find corresponding frame data
                t = runtime[j]
                frame_data = None
                for frame in self.data["state"]:
                    if abs(frame["runtime"] - t) < 0.01:  # Match runtime
                        frame_data = frame
                        break

                if frame_data:
                    for robot in frame_data["robots"]:
                        if robot["id"] == robot_id:
                            if robot["opt"].get("status") == "failed":
                                error_msg = robot["opt"].get("error", "").upper()
                                if "INFEASIBLE" in error_msg:
                                    infeasible_times.append(t)
                                elif "UNBOUNDED" in error_msg:
                                    unbounded_times.append(t)
                                else:
                                    other_error_times.append(t)
                            else:
                                # Success case
                                success_times.append(t)

            # Plot successes as light green background (very transparent)
            if success_times:
                for t in success_times:
                    ax.axvspan(t - 0.1, t + 0.1, ymin=(i)/num_robots, ymax=(i+1)/num_robots,
                              color='lightgreen', alpha=0.3, zorder=1)

            # Plot failures as discrete markers
            if infeasible_times:
                ax.scatter(infeasible_times, [i] * len(infeasible_times),
                          c='orange', s=80, marker='x', linewidth=2, zorder=3)
            if unbounded_times:
                ax.scatter(unbounded_times, [i] * len(unbounded_times),
                          c='purple', s=80, marker='^', linewidth=1.5, zorder=3)
            if other_error_times:
                ax.scatter(other_error_times, [i] * len(other_error_times),
                          c='red', s=80, marker='s', linewidth=1.5, zorder=3)

        # Add subtle grid lines to separate robot rows
        for i in range(1, num_robots):
            ax.axhline(y=i - 0.5, color='gray', linestyle='--', alpha=0.3, linewidth=0.5)

        # Set ticks and labels
        ax.set_yticks(range(num_robots))
        ax.set_yticklabels([f'Robot {rid}' for rid in robot_ids])
        ax.set_xlabel(self.xlabel)
        ax.set_ylabel(self.ylabel)
        ax.set_title(self.title)

        # Set y-axis limits
        ax.set_ylim(-0.5, num_robots - 0.5)

        # Add grid for better readability
        ax.grid(True, alpha=0.2, axis='x')

        # Create legend by checking actual error messages in the data
        legend_elements = []
        has_infeasible = False
        has_unbounded = False
        has_other_error = False

        # Check error messages in the original data
        for frame in self.data["state"]:
            for robot in frame["robots"]:
                if robot["opt"].get("status") == "failed":
                    error_msg = robot["opt"].get("error", "").upper()
                    if "INFEASIBLE" in error_msg:
                        has_infeasible = True
                    elif "UNBOUNDED" in error_msg:
                        has_unbounded = True
                    else:
                        has_other_error = True

        if has_infeasible:
            legend_elements.append(mlines.Line2D([], [], marker='x', linestyle='None', color='orange',
                                               markersize=8, label='Infeasible'))
        if has_unbounded:
            legend_elements.append(mlines.Line2D([], [], marker='^', linestyle='None', color='purple',
                                               markersize=8, label='Unbounded'))
        if has_other_error:
            legend_elements.append(mlines.Line2D([], [], marker='s', linestyle='None', color='red',
                                               markersize=8, label='Other Error'))

        # Always add success if showing legend
        from matplotlib.patches import Rectangle
        legend_elements.append(Rectangle((0, 0), 1, 1, fc='lightgreen', alpha=0.3, label='Success'))

        if self.show_legend and legend_elements:
            ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.15, 1), framealpha=0.9)

        # Adjust layout to make room for legend
        if self.show_legend:
            self.fig.tight_layout(rect=[0, 0, 0.85, 1])
        else:
            self.fig.tight_layout()

    def update(self, frame):
        """Animation support - could be extended to show real-time failures"""
        if hasattr(self, 'runtime') and len(self.runtime) > 0:
            # Implementation for animation if needed
            pass