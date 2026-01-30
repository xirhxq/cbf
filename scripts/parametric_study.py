#!/usr/bin/env python3
"""
Parametric Study Script

This script runs multiple simulations with different parameter values
and saves all results in a time-stamped folder structure.

Usage:
    python parametric_study.py --param searching.front-sector.outer-radius --values 200 300 400 500
    python parametric_study.py --param cbfs.without-slack.comm-fixed.max-range --values 600 700 800 900
"""

import argparse
import json
import os
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Any, List

# Get the absolute path of the script directory
SCRIPT_DIR = Path(__file__).parent.resolve()
PROJECT_ROOT = SCRIPT_DIR.parent


def get_timestamp() -> str:
    """Get current timestamp in YYYY-MM-DD_HH-MM-SS format."""
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def set_nested_value(obj: dict, path: str, value: Any) -> dict:
    """Set a value in a nested dictionary using dot notation path."""
    keys = path.split('.')
    current = obj

    for key in keys[:-1]:
        if key not in current:
            current[key] = {}
        current = current[key]

    current[keys[-1]] = value
    return obj


def get_nested_value(obj: dict, path: str, default=None) -> Any:
    """Get a value from a nested dictionary using dot notation path."""
    keys = path.split('.')
    current = obj

    for key in keys:
        if isinstance(current, dict) and key in current:
            current = current[key]
        else:
            return default

    return current


def run_simulation(config_path: str, output_dir: str) -> bool:
    """
    Run a single simulation with the given config file.

    Args:
        config_path: Path to the config file
        output_dir: Directory where simulation will save its data

    Returns:
        True if simulation succeeded, False otherwise
    """
    print(f"\n{'='*60}")
    print(f"Running simulation with config: {config_path}")
    print(f"Output directory: {output_dir}")
    print(f"{'='*60}\n")

    try:
        # Use cmake-build-release directory
        build_dir = PROJECT_ROOT / "cmake-build-release"
        result = subprocess.run(
            ["./Swarm", str(Path(config_path).resolve())],
            cwd=str(build_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=7200  # 2 hour timeout
        )

        if result.returncode == 0:
            print(f"✓ Simulation completed successfully")
            return True
        else:
            print(f"✗ Simulation failed with return code {result.returncode}")
            print(f"STDOUT: {result.stdout}")
            print(f"STDERR: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        print(f"✗ Simulation timed out after 2 hours")
        return False
    except Exception as e:
        print(f"✗ Error running simulation: {e}")
        return False


def create_summary(output_dir: str, param_name: str, param_values: List[Any],
                   results: List[dict]) -> None:
    """
    Create a summary JSON file with parametric study results.

    Args:
        output_dir: Directory to save summary
        param_name: Name of the parameter that was varied
        param_values: List of parameter values tested
        results: List of result dictionaries for each run
    """
    summary = {
        "parametric_study": {
            "parameter_name": param_name,
            "parameter_values": param_values,
            "num_runs": len(results),
            "timestamp": get_timestamp(),
            "results": results
        }
    }

    summary_path = Path(output_dir) / "summary.json"
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)

    print(f"\n✓ Summary saved to: {summary_path}")


def parse_value(value_str: str, value_type: str) -> Any:
    """Parse a string value to the specified type."""
    if value_type == 'int':
        return int(value_str)
    elif value_type == 'float':
        return float(value_str)
    elif value_type == 'bool':
        return value_str.lower() in ['true', '1', 'yes']
    else:
        return value_str


def main():
    parser = argparse.ArgumentParser(
        description="Run parametric study with varying parameter values"
    )
    parser.add_argument(
        "--param",
        type=str,
        required=True,
        help="Parameter path in dot notation (e.g., 'searching.front-sector.outer-radius')"
    )
    parser.add_argument(
        "--values",
        type=str,
        nargs='+',
        required=True,
        help="List of parameter values to test (space-separated)"
    )
    parser.add_argument(
        "--base-config",
        type=str,
        default="../config/config.json",
        help="Base config file path (default: ../config/config.json)"
    )
    parser.add_argument(
        "--type",
        type=str,
        choices=['int', 'float', 'bool', 'str'],
        default='float',
        help="Type of parameter values (default: float)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory (default: ../data/TIMESTAMP)"
    )

    args = parser.parse_args()

    # Parse parameter values
    param_values = [parse_value(v, args.type) for v in args.values]

    # Create output directory in data_multi (separate from regular data)
    if args.output_dir is None:
        output_dir = PROJECT_ROOT / "data_multi" / get_timestamp()
    else:
        output_dir = Path(args.output_dir)

    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Parametric study output directory: {output_dir}")

    # Load base config
    with open(args.base_config, 'r') as f:
        base_config = json.load(f)

    # Get original parameter value
    original_value = get_nested_value(base_config, args.param)
    print(f"Original {args.param} = {original_value}")

    # Run simulations for each parameter value
    results = []

    for i, value in enumerate(param_values):
        print(f"\n{'#'*60}")
        print(f"Run {i+1}/{len(param_values)}: {args.param} = {value}")
        print(f"{'#'*60}")

        # Create modified config
        config = json.loads(json.dumps(base_config))  # Deep copy
        config = set_nested_value(config, args.param, value)

        # Set output_path so Swarm saves to output_dir/TIMESTAMP/
        config['output_path'] = str(output_dir)

        # Save config to temp file
        temp_config_path = output_dir / f"temp_config_{i}.json"
        with open(temp_config_path, 'w') as f:
            json.dump(config, f, indent=2)

        # Get absolute path for C++ program
        abs_config_path = str(temp_config_path.resolve())

        # Run simulation (Swarm will save to output_dir/TIMESTAMP/data.json)
        success = run_simulation(abs_config_path, str(output_dir.resolve()))

        if success:
            # Find the timestamp subfolder that Swarm created inside output_dir
            # Get all timestamp subfolders and sort by name (most recent last)
            run_subfolders = sorted([f for f in output_dir.iterdir()
                                    if f.is_dir() and not f.name.startswith('temp')],
                                   key=lambda x: x.name)

            if run_subfolders:
                # Take the most recently created subfolder
                swarm_folder = run_subfolders[-1]
                print(f"  ✓ Data saved to: {swarm_folder}")

                # Read results from data.json
                data_path = swarm_folder / "data.json"
                if data_path.exists():
                    with open(data_path, 'r') as f:
                        data = json.load(f)

                    state = data.get('state', [])
                    final_runtime = state[-1]['runtime'] if state else 0

                    # Calculate final coverage and find completion time
                    para = data.get('para', {})
                    grid_world = para.get('gridWorld', {})
                    x_num = grid_world.get('xNum', 1)
                    y_num = grid_world.get('yNum', 1)
                    total_cells = x_num * y_num

                    searched_cells = set()
                    completion_time = final_runtime  # Default to final runtime

                    for frame in state:
                        if "update" in frame and len(frame["update"]) > 0:
                            for grid in frame["update"]:
                                cell_id = (grid[0], grid[1])
                                searched_cells.add(cell_id)

                        # Check if coverage just reached 100%
                        current_coverage = len(searched_cells) / total_cells * 100 if total_cells > 0 else 0
                        if current_coverage >= 99.9 and completion_time == final_runtime:
                            completion_time = frame['runtime']

                    final_coverage = len(searched_cells) / total_cells * 100 if total_cells > 0 else 0

                    results.append({
                        "parameter_value": value,
                        "duration": completion_time,  # Time to reach 100% coverage
                        "final_coverage": final_coverage,
                        "data_folder": swarm_folder.name
                    })

                    print(f"  ✓ Completion time: {completion_time:.1f}s, Final coverage: {final_coverage:.1f}%")
                else:
                    results.append({
                        "parameter_value": value,
                        "duration": 0,
                        "final_coverage": 0,
                        "data_folder": swarm_folder.name,
                        "error": "data.json not found"
                    })
                    print(f"  ✗ data.json not found")
            else:
                print(f"  ✗ No data folder created in {output_dir}")
                results.append({
                    "parameter_value": value,
                    "duration": 0,
                    "final_coverage": 0,
                    "error": "No data folder created"
                })
        else:
            results.append({
                "parameter_value": value,
                "duration": 0,
                "final_coverage": 0,
                "error": "Simulation failed"
            })

        # Comment out temp config cleanup for debugging
        # temp_config_path.unlink()

        # Small delay between runs
        time.sleep(1)

    # Create summary
    create_summary(str(output_dir), args.param, param_values, results)

    print(f"\n{'='*60}")
    print("Parametric study completed!")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
