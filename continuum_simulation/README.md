# Continuum robot simulation (Python)

First-pass model of the tendon-driven continuum robot that carries the pipe plug and delivers disinfectant through a hospital sink P-trap. It is used to compare continuum designs (length, number and length of sections, spring backbone, disks, tendons) against a range of drain geometries before building hardware.

This is **Tier 1a** of the simulation plan: a fast, kinematic model for design screening. A Cosserat rod model (Tier 1b) and a combined KUKA + continuum + drain simulation with wall contact (Tier 3, MuJoCo) will follow.

## Files

| File | What it does |
|---|---|
| `pcc_model.py` | Piecewise constant-curvature (PCC) model of the continuum inside a parametric P-trap. Fits the robot to the drain at each insertion depth, checks wall clearance and minimum bend radius, computes tendon tensions, and makes the figures and animation. |
| `compare_sections.py` | Compares designs with custom section lengths (e.g. equal vs. matched to trap features). |
| `requirements.txt` | Python dependencies. |
| `../continuum_sim_parameters.xlsx` | Parameter workbook shared with the KUKA study (repo root). |

## Setup

Python 3.10 or newer.

```
python -m pip install -r requirements.txt
```

## Running

From this folder:

```
python pcc_model.py --params ../continuum_sim_parameters.xlsx
```

Options:

- `--params <file>`: read baseline values from the workbook. Without it, the defaults in the `Design` and `Drain` classes are used.
- `--out <folder>`: where figures go (default `figures/` next to the script).
- `--no-gif`: skip the insertion animation (faster).

Close the workbook in Excel before running. Excel locks open files.

### Custom section lengths

Edit the `designs` dictionary in `compare_sections.py`, then run `python compare_sections.py`. Lengths are listed base to tip, in mm, and their sum is the robot's total length (the workbook's `L` is ignored for these designs).

## Inputs

All inputs come from the **Baseline** column of the workbook:

- **Design Inputs:** length, sections, spring (wire diameter, coil diameter, pitch, material), disks (spacing, OD, thickness), tendons (count, offset radius, friction, max tension), plug, sensors.
- **Drain Geometry:** pipe ID, tailpiece length, U-bend radius, water seal depth, trap arm length. Outlet height is derived from the **Sink** sheet.

Values flagged yellow in the workbook are placeholder estimates until measured.

## Outputs (`figures/`)

| File | Shows |
|---|---|
| `1_snapshots.png` | Best-fit robot shape in the P-trap at four insertion depths (3 sections). |
| `2_sections_vs_conformance.png` | Maximum deviation from the drain centreline vs. insertion depth for 1–8 equal sections, against the radial clearance. Above the dashed line, the robot would touch the wall. |
| `3_tendon_tension.png` | Actuator-side tendon tension per section vs. depth, including capstan friction. |
| `4_design_chart.png` | Tension needed to bend to the trap radius vs. spring wire diameter, for several tendon offsets. |
| `insertion.gif` | Animation of the insertion (3 sections). |

The script also prints the spring bending stiffness, minimum bend radius, radial clearance, the deepest wall-free insertion for each section count, and a 3-D kinematics check.

## Model assumptions

Keep these in mind when reading the results:

- **Free space:** walls are drawn but do not push on the robot. A "hits wall" result means the robot would need wall contact to get through, not that the design fails.
- **Constant curvature:** each section bends as a single circular arc, so a section spanning a straight pipe and a bend must compromise.
- **Planar:** the fit is 2-D in the plane of the trap. 3-D kinematics and tendon-length mapping are implemented (`pcc_transform`, `tendon_displacements`) but not yet used in the fit.
- **Stiffness affects tension, not shape.** Spring bending stiffness uses Wahl's helical-spring formula; the fluid line, inflation line and sensors are not included.
- **Tendon friction** uses the capstan equation with a single friction coefficient.

## Known limitations and next steps

- No wall contact, insertion force or buckling. These need the Cosserat model and the MuJoCo simulation.
- Non-uniform stiffness is not modelled.
- The plug's length and stiffness at the tip are not yet included in the fit.
- All parameters are placeholders until the prototype is measured.

## References

- Webster, R. J., and Jones, B. A. (2010). Design and kinematic modeling of constant curvature continuum robots: A review. *International Journal of Robotics Research*, 29(13).
- Rucker, D. C., and Webster, R. J. (2011). Statics and dynamics of continuum robots with general tendon routing and external loading. *IEEE Transactions on Robotics*, 27(6).
