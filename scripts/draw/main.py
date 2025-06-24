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
        print(f"[{idx}]: {option}")
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
            'name': 'Animation (Full Set)',
            'action': lambda: AnimationDrawer(files).run_animation(['map', 'opt', 'fix', 'cbf'])
        }
    ]

    interactive_menu(menu_options)

