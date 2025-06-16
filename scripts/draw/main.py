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
            'name': 'Generate Search Heatmap',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['sh'])
        },
        {
            'name': 'Generate Energy Level (All Robots in One)',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['energy'])
        },
        {
            'name': 'Generate Statistics',
            'action': lambda: StaticGlobalPlotDrawer(files).draw_plots(['sh', 'energy'])
        },
        {
            'name': 'Generate CBF Values per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['cbf'])
        },
        {
            'name': 'Generate CVT CBF Value per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['cvt'])
        },
        {
            'name': 'Generate Min CBF Value per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['min'])
        },
        {
            'name': 'Generate Fixed Communication Range Plots',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['fix'])
        },
        {
            'name': 'Generate Heatmap per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['heat'])
        },
        {
            'name': 'Generate Statistic per Robot',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['energy', 'min'])
        },
        {
            'name': 'Generate Combined Plot (Fix + CBF)',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['fix', 'cbf'])
        },
        {
            'name': 'Generate CVT CBF Value Combined Plot',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['cvt'])
        },
        {
            'name': 'Generate Energy Level (Grouped Subplots)',
            'action': lambda: StaticGroupPlotDrawer(files).draw_plots(['energy'])
        },
        {
            'name': 'Generate Energy Level (Each Robot Separately)',
            'action': lambda: StaticSeparatePlotDrawer(files).draw_plots(['energy'])
        },
        {
            'name': 'Run Animation (Map Only)',
            'action': lambda: AnimationDrawer(files).run_animation(['map'])
        },
        {
            'name': 'Run Animation (Full Set)',
            'action': lambda: AnimationDrawer(files).run_animation(['map', 'opt', 'fix', 'cbf'])
        }
    ]

    interactive_menu(menu_options)

