from dataclasses import replace
import numpy as np
import pcc_model as m

from pathlib import Path
HERE = Path(__file__).parent
des, drn = m.load_params(HERE / "continuum_sim_parameters.xlsx")
path = m.DrainPath(drn, ext=des.L + 50)

designs = {
    "equal 4":     [125, 125, 125, 125],
    "matched 4":   [200, 126, 93, 81],      # tail | U-bend | rise+90° | arm
    "matched 5":   [200, 126, 30, 63, 81],  # tail | U-bend | rise | 90° | arm
    "tip-heavy 4": [275, 75, 75, 75],
}

for name, lens in designs.items():
    d = replace(des, section_lengths=lens)
    reach = 0
    for depth in np.linspace(20, sum(lens), 60):
        f = m.fit_robot(d, path, depth)
        if not f.feasible:
            break
        reach = depth
    print(f"{name:12s} reach {reach:4.0f} mm")