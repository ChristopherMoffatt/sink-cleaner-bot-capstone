# KUKA reachability study (MATLAB)

Checks whether the KUKA LBR iiwa 7 R800, mounted on the Ewellix lift on the Clearpath Ridgeback, can hold the continuum robot pointing straight down over a sink drain and insert it, across the range of sinks in the project scope.

It compares two insertion strategies:

- **Mode A:** the lift stays fixed and the **arm** performs the insertion stroke.
- **Mode B:** the arm holds one pose and the **lift** performs the insertion stroke.

## Files

| File | What it does |
|---|---|
| `kuka_reachability.m` | The study: sweeps lift position and drain position, solves inverse kinematics along the insertion stroke, checks collisions with the sink, faucet and wall, and saves plots, snapshots and animations. |
| `que36.urdf` | Full robot model (Ridgeback, Ewellix lift, iiwa 7) exported from the robot. |
| `../continuum_sim_parameters.xlsx` | Parameter workbook shared with the continuum model. The script also accepts it in this folder. |
| `que36_pkgs/` | **Not in the repo.** Mesh files; see below. |

## Requirements

- MATLAB R2025a (earlier recent releases may work)
- Robotics System Toolbox

## Getting the mesh files (optional, recommended)

Without meshes, the script still runs using kinematics only, with the arm approximated as cylinders for collision checks. With meshes, it draws the full robot and runs a full collision check.

The meshes come from the robot itself and are not in the repo (large, and partly Clearpath/KUKA files). With the robot powered on and your laptop connected over Ethernet, as described in the Clearpath quickstart guide:

```bash
# on the robot (over SSH)
cd ~
PKGS=$(grep -o 'package://[^/]*' que36.urdf | sed 's#package://##' | sort -u)
tar czf que36_pkgs.tgz -C / $(for p in $PKGS; do rospack find $p | sed 's#^/##'; done)
```

Copy `que36_pkgs.tgz` to your laptop with `scp`, then extract it next to the script:

```
mkdir que36_pkgs
tar xzf que36_pkgs.tgz -C que36_pkgs
```

If the robot's `que36.urdf` is missing, export it first (on the robot):

```bash
python3 -c "import rosparam; print(rosparam.get_param('/robot_description'))" > ~/que36.urdf
```

## Running

Open `kuka_reachability.m` in MATLAB and run it. Close the workbook in Excel first. With meshes, a run takes several minutes.

At the top of the script you can change the sweep resolution: `nLift`, `nDrain` and `nStep`.

## Inputs

From the workbook:

- **Sink sheet:** rim height, bowl depth, sink depth and width, bowl opening, drain position (front edge to drain), faucet position, wall-mounted or freestanding, Ridgeback parking gap, actuator pack size, collision margins.
- **Design Inputs sheet:** continuum length `L` and disk OD.
- **Quick Checks sheet:** plug target depth, used to warn if the continuum is too short.

From the URDF: lift travel, iiwa joint limits, and the KUKA's mounting position on the Ridgeback.

Scope limits from the project definition are built into the Sink sheet: sink depth at most 24 in (610 mm), rim at most 36 in (914 mm), faucet not directly above the drain, 1-1/2 in pipe.

## Outputs (`reach_results/`)

| File | Shows |
|---|---|
| `modeA_map.png` | Lift position vs. drain distance with the arm doing the insertion. Colour is the smallest joint-limit margin along the stroke; grey cells failed (IK = unreachable, C = collision, flip = the arm would need to jump configurations mid-stroke). |
| `modeB_margin.png` | Joint-limit margin vs. drain distance with the lift doing the insertion. |
| `modeA_snapshots.png`, `modeB_snapshots.png` | Robot at the start, halfway and end of insertion over the baseline sink. |
| `modeA_insertion.gif`, `modeB_insertion.gif` | Animations of the insertion. |
| `reach_results.mat` | All results, for further analysis. |

Status codes in the command window: 0 ok, 1 unreachable, 2 collision, 3 joint flip mid-stroke.

## How it works

- **Frame:** Ridgeback `base_link`, x forward toward the sink, z up. The floor is 75.9 mm below `base_link` (wheel radius from the URDF).
- **Insertion stroke:** the flange points straight down above the drain. The stroke ends with the actuator pack one clearance above the outlet.
- **Redundancy:** the iiwa has 7 joints for a 6-DOF pose (tool roll is left free), so many arm configurations reach each pose. The script collects up to 30 solutions, keeps the 5 with the most room before any joint limit, runs the full stroke from each, and keeps the best complete stroke. Seeds are fixed (`rng(1)`), so results are repeatable.
- **Sink:** modelled as boxes around a rectangular bowl opening, plus a faucet spout and an optional wall.

## Limitations

- Sink dimensions are placeholders until the hospital provides real ones. The drain's distance from the sink front (relative to the AprilTag) is the most important one.
- The continuum is drawn only above the outlet; its path through the trap comes from the continuum model.
- No payload torques, dynamics or AprilTag localization error yet.
- The lift's travel (500 mm in the URDF) should be checked against the measured range on the robot.
