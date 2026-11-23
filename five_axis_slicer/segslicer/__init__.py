"""segslicer - a "3+2" segmented planar slicer for a DIY 5-axis FDM printer.

Modules (milestone 1):
    config      - load machine.yaml / settings.yaml into dataclasses
    transforms  - rotation helpers (direction -> +Z and back)
    kinematics  - A/C inverse kinematics, part <-> machine coordinates
    collision   - hotend cone + cylinder model and collision checks
    mesh_io     - STL loading and placing parts on the table
    viewer      - Polyscope GUI
"""

__version__ = "0.1.0"
