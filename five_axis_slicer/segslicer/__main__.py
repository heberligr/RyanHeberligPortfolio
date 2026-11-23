"""Run the app:  python -m segslicer [model.stl] [--machine path/to/machine.yaml]"""

from __future__ import annotations

import argparse

from .config import DEFAULT_MACHINE_PATH, load_machine


def main() -> None:
    parser = argparse.ArgumentParser(description="Segmented 3+2 slicer for a 5-axis FDM printer")
    parser.add_argument("model", nargs="?", help="STL file to open")
    parser.add_argument("--machine", default=str(DEFAULT_MACHINE_PATH), help="machine.yaml to use")
    args = parser.parse_args()

    machine = load_machine(args.machine)
    print(f"Machine: {machine.name} ({machine.firmware}), pivot {machine.kinematics.pivot.tolist()}")

    # Import here so `--help` works even if the GUI library has issues.
    from .viewer import SlicerViewer

    SlicerViewer(machine, args.model).run()


if __name__ == "__main__":
    main()
