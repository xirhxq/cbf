from utils import *
from drawers.drawers import *

def interactive_menu(options):
    while True:
        print("\n" + "-" * 30)
        for idx, option in enumerate(options):
            print(f"[{idx}]: {option['name']}")
        print("[q]: Quit")
        print("[a]: Run All")
        choice = input("Choose an option: ").strip().lower()

        if choice == 'q':
            print("Exiting...")
            break
        elif choice == 'a':
            print("\nRunning all options...\n")
            for option in options:
                print(f"Running: {option['name']}...")
                option['action']()
            print("\nAll tasks completed.")
            break
        elif choice.isdigit() and 0 <= int(choice) < len(options):
            selected = options[int(choice)]
            print(f"\nRunning: {selected['name']}...\n")
            selected['action']()
        else:
            print("Invalid input. Please try again.")


def interactive_selection(options):
    print("Select options:")
    for idx, option in enumerate(options):
        file_path = option
        try:
            import json
            with open(file_path, 'r') as f:
                data = json.load(f)

            config = data.get('config', {})
            state = data.get('state', [])

            # Basic info
            duration = state[-1]['runtime'] if state else 0
            exec_mode = config.get('execute', {}).get('execution-mode', 'unknown')[:4]  # dist/cen
            num_robots = config.get('num', 0)
            file_name = os.path.basename(os.path.dirname(file_path))

            # Searching method with parameters
            searching = config.get('searching', {})
            method_full = searching.get('method', 'unknown')
            # Abbreviate: downward->down, front-sector->sect, front-cone->cone
            if method_full == 'downward':
                searching_method = 'down'
            elif method_full == 'front-sector':
                searching_method = 'sect'
            elif method_full == 'front-cone':
                searching_method = 'cone'
            else:
                searching_method = method_full[:4]
            searching_params = ''
            if searching_method == 'down':
                radius = searching.get('downward', {}).get('radius', 0)
                searching_params = f'(r{int(radius)})'
            elif searching_method == 'sect':
                inner_r = searching.get('front-sector', {}).get('inner-radius', 0)
                outer_r = searching.get('front-sector', {}).get('outer-radius', 0)
                half_angle = searching.get('front-sector', {}).get('half-angle-deg', 0)
                searching_params = f'(r{int(inner_r)}-{int(outer_r)},{int(half_angle)}°)'

            # Calculate final search percentage
            grid_world = data.get('para', {}).get('gridWorld', {})
            x_num = grid_world.get('xNum', 1)
            y_num = grid_world.get('yNum', 1)
            total_cells = x_num * y_num
            searched_cells = set()
            for frame in state:
                if "update" in frame and len(frame["update"]) > 0:
                    for grid in frame["update"]:
                        cell_id = (grid[0], grid[1])
                        searched_cells.add(cell_id)
            search_pct = len(searched_cells) / total_cells * 100 if total_cells > 0 else 0

            # CBF info
            cbfs = config.get('cbfs', {})
            cbf_info = []
            if cbfs.get('without-slack', {}).get('comm-fixed', {}).get('on', False):
                max_range = cbfs.get('without-slack', {}).get('comm-fixed', {}).get('max-range', 0)
                cbf_info.append(f'comm(r{int(max_range)})')
            cvt_on = cbfs.get('with-slack', {}).get('cvt', {}).get('on', False)
            if cvt_on:
                exploration_mode = cbfs.get('with-slack', {}).get('cvt', {}).get('exploration-mode', 'unknown')
                exploration_mode_short = exploration_mode.split('-')[0][:6]
                cbf_info.append(f'cvt({exploration_mode_short})')
            if cbfs.get('without-slack', {}).get('safety', {}).get('on', False):
                cbf_info.append('safety')

            print(f"[{idx}]: {file_name} - {duration:.1f}s | {search_pct:.1f}% | {exec_mode} | {num_robots}r | {searching_method}{searching_params} | {', '.join(cbf_info) if cbf_info else 'none'}")
        except:
            print(f"[{idx}]: {os.path.basename(os.path.dirname(file_path))} - Error reading file")

    choice = input("Choose options (separated by comma): ").strip().lower()
    selected_options = [options[int(choice)] for choice in choice.split(',')]
    return selected_options


def findNewestFiles(folder: str, ptn: str, num: int = 1):
    directories = glob.glob(os.path.join(folder, ptn))
    assert len(directories) > 0, "No directory found with pattern {}".format(ptn)
    directories = [d for d in directories if len(glob.glob(d + '/data.json')) > 0]
    num = min(num, len(directories))
    return [os.path.join(d, 'data.json') for d in sorted(directories, reverse=True)[:num]]


if __name__ == '__main__':
    files = interactive_selection(findNewestFiles('../../data', '*', 10))

    menu_options = [
        {
            'name': 'Search Heatmap',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['sh'])
        },
        {
            'name': 'Energy Level, All Robots',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['energy'])
        },
        {
            'name': 'Search Heatmap & Energy, All Robots',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['sh', 'energy'])
        },
        {
            'name': 'CBF Values, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['cbf'])
        },
        {
            'name': 'CBF Values, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['cbf-energy'], time_range=(15, 25), id_list=[1])
        },
        {
            'name': 'CVT CBF Value, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['cvt'])
        },
        {
            'name': 'Min CBF Value, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['min'])
        },
        {
            'name': 'Fixed Communication Range, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['fix'])
        },
        {
            'name': 'Heatmap, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['heat'])
        },
        {
            'name': 'Energy & Min CBF, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['energy', 'min'])
        },
        {
            'name': 'Fix Communication Range & CBF & Energy, Grouped',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['fix', 'cbf', 'energy'])
        },
        {
            'name': 'CVT CBF, Grouped',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['cvt'])
        },
        {
            'name': 'Energy Level, Grouped',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['energy'])
        },
        {
            'name': 'Energy Level, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['energy'])
        },
        {
            'name': 'Control Input, All Robots',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['u'])
        },
        {
            'name': 'Control Input, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['u'])
        },
        {
            'name': 'Control Input, Grouped',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['u'])
        },
        {
            'name': 'Animation (Map)',
            'action': lambda: AnimationDrawer(files).run_animation(['map'])
        },
        {
            'name': 'Animation (Map, Last 5 Seconds)',
            'action': lambda: AnimationDrawer(files).run_animation(['map'], last_seconds=5)
        },
        {
            'name': 'Animation (Map, Certain Time Range)',
            'action': lambda: AnimationDrawer(files).run_animation(['map'], time_range=(130, 140))
        },
        {
            'name': 'Animation (Full Set)',
            'action': lambda: AnimationDrawer(files).run_animation(['map', 'opt-ct', 'cbf'])
        },
        {
            'name': 'Animation (Full Set, Last 5 Seconds)',
            'action': lambda: AnimationDrawer(files).run_animation(['map', 'opt-ct'], last_seconds=5)
        },
        {
            'name': 'Animation (Full Set, Certain Time Range)',
            'action': lambda: AnimationDrawer(files).run_animation(['map', 'opt-ct'], time_range=(120, 130))
        },
        {
            'name': 'Animation (Full Set, Certain Time Range, #1 Only)',
            'action': lambda: AnimationDrawer(files).run_animation(
                ['map', 'opt-vec', 'cbc-energy', 'cbf-energy', 'opt-ct'],
                time_range=(130, 140),
                id_list=[1]
            )
        },
        {
            'name': 'Animation (Map with CVT CBF)',
            'action': lambda: AnimationDrawer(files).run_animation(['map', 'cvt'])
        },
        {
            'name': 'CBF Derivative, Certain Time Range',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(
                ['cbc-energy'],
                time_range=(130, 140),
                id_list=[1]
            )
        },
        {
            'name': 'CBF Derivative, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['cbc-energy'])
        },
        {
            'name': 'Search Percentage Over Time, All Robots',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['sp'])
        },
        {
            'name': 'Centralized CBF Values',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['centralized-cbf'])
        },
        {
            'name': 'Centralized Communication CBF',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['centralized-comm'])
        },
        {
            'name': 'Centralized CVT CBF',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['centralized-cvt'])
        },
        {
            'name': 'CommCBF Distance + Uncertainty',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['comm-uncertainty'])
        },
        {
            'name': 'CommCBF Distance + Uncertainty (From Max Range)',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['comm-uncertainty-maxrange'])
        },
        {
            'name': 'Optimization Failure Timeline',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['optimization-failure'])
        },
        {
            'name': 'Animation (Centralized CBF)',
            'action': lambda: AnimationDrawer(files).run_animation(['centralized-cbf'])
        },
        {
            'name': 'Animation (Centralized Communication)',
            'action': lambda: AnimationDrawer(files).run_animation(['centralized-comm'])
        },
        {
            'name': 'CVT Center Density Over Time, All Robots',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['cvt-center-density'])
        },
        {
            'name': 'CVT Center Density Over Time, Grouped',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['cvt-center-density'])
        },
        {
            'name': 'CVT Center Density Over Time, Per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['cvt-center-density'])
        },
        {
            'name': 'Position Uncertainty Evolution (Global)',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['position-uncertainty'])
        },
        {
            'name': 'Position Uncertainty Distribution (Global)',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['uncertainty-heatmap'])
        },
        {
            'name': 'Localization Constraints h_loc (Global)',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['h_loc'])
        },
        {
            'name': 'Safety Constraints h_safe (Global)',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['h_safe'])
        },
        {
            'name': 'CBF Analysis - Constraint CBFs (Per Robot)',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['constraint_cbfs'])
        },
        {
            'name': 'CBF Analysis - Task CBFs (Per Robot)',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['task_cbfs'])
        },
        {
            'name': 'CBF Analysis - Slack Variables (Per Robot)',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['slack_vars'])
        }
    ]

    interactive_menu(menu_options)

