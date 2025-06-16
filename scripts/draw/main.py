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


if __name__ == '__main__':
    files = [findNewestFile('../../data', '*')]

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
            'name': 'Fix Communication Range & CBF, Grouped',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['fix', 'cbf'])
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
            'name': 'Animation (Map)',
            'action': lambda: AnimationDrawer(files).run_animation(['map'])
        },
        {
            'name': 'Animation (Full Set)',
            'action': lambda: AnimationDrawer(files).run_animation(['map', 'opt', 'fix', 'cbf'])
        }
    ]

    interactive_menu(menu_options)

