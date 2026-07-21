# Manual Mocap Figure-8

This is the current working AIMSLab flight workflow. It uses the Crazyflie's
low-level commander: the pilot owns takeoff and landing thrust, while the
script uses OptiTrack/VRPN pose data to assist X/Y position, yaw, the 3 ft
pre-figure-8 climb, and figure-8 altitude correction.

The flight script is [`mocap_manual_thrust_assisted_figure8.py`](mocap_manual_thrust_assisted_figure8.py).
Its configuration lives at the top of that file. Do not copy constants from
older handoff notes without checking the script first.

## Setup

Install this repository's Python dependencies, including `motioncapture`.
`matplotlib` is optional but enables automatic 3D path images after each run.

Before a powered run:

- Close `cfclient` and any other program using the Crazyradio.
- Confirm Motive tracks the configured rigid body and VRPN is available.
- Check that the cage is clear and that an operator has a physical power-cut
  option ready.
- Confirm the URI, VRPN host, rigid-body name, and cage-corner values near the
  top of the flight script match the lab setup.

Run from the repository root:

```bash
python3 mocap_manual_thrust_assisted_figure8.py
```

The script writes a CSV to `flight_logs/` and, when matplotlib is available,
writes a matching `-3d-path.png` image. These are local experiment artifacts;
they are not committed to Git.

While the flight runs, the script starts `plot_crazyflie_3d_track.py` as a live
viewer and asks it to save the final 3D path image when the flight loop ends.
The console prints both output paths. If matplotlib is unavailable in the
plotter's Python environment, the flight still completes and the console
reports that the image could not be produced.

## Proven Flight Sequence

This sequence was established by the successful 2026-07-13 manual baseline
and remains the required entry sequence for the current faster path profile:

1. Arm when prompted, then press `R` to ramp to ready thrust.
2. Establish a low, stable hover.
3. Press `T` to enable the 3 ft height helper. Wait until the on-screen status
   reports `3ft ready: YES`.
4. Press `F` once to start the figure-8 from the current position and height.
5. Press `F` again to stop the path, return to the figure-8 start point, and
   begin landing.

The figure-8 only starts when mocap is fresh and the drone is sufficiently
settled in horizontal and vertical speed. This is deliberate: starting while
the drone is still climbing produces a distorted path.

## Current Verified Baseline

### 2026-07-21 Fast Compact Figure-8

The current manual-flight baseline is a compact, fast figure-8. The script is
the source of truth for its active constants; at this update the relevant
settings are:

- `FIGURE8_NOMINAL_SPEED_SCALE = 5.70`, which is a `4.2 s` nominal path-clock
  period from the `24 s` reference profile.
- `FIGURE8_MAX_ANGLE_DEG = 26.0` during an airborne figure-8.
- `FIGURE8_TARGET_VELOCITY_FEEDFORWARD = 0.70` so the X/Y controller follows a
  moving target instead of reacting only after position error appears.
- `FIGURE8_SPEEDUP_SCALE_PER_S = 1.00`; the path clock recovers more promptly
  after a slow section.
- Figure-8 altitude correction is assisted by tilt-thrust compensation, capped
  at `3400 raw` in addition to the normal Z PID correction.
- The shared center crossing uses a cubic vertical profile (`sin(phase)^3`).
  It preserves the same path extents while removing the prior abrupt vertical
  curvature reversal between the top and bottom lobes.

These settings were progressed through a sequence of reviewed successful
flights. The latest reviewed log,
`flight_logs/mocap-assisted-figure8-20260721-100041.csv`, completed the
figure-8 and entered normal return/descent with no stale-mocap or safety event.
During its figure-8 phase it recorded:

| Metric | Result |
| --- | ---: |
| Horizontal speed, median / p95 / peak | `2.13 / 2.76 / 3.16 m/s` |
| Target error, median / p95 / peak | `0.52 / 0.79 / 0.86 m` |
| Height above start, median / p95 | `1.00 / 1.12 m` |
| Pitch or roll cap reached | `1.5%` of figure-8 samples |
| Tilt compensation at its cap | `8.6%` of figure-8 samples |

The nominal speed is not a promise that every part of the path advances at
that rate. The script slows the path clock when tracking error grows or when
cage clearance narrows. In the latest run the path clock was below `3.0x` for
about `81%` of figure-8 samples. This is intentional: tracking and cage margin
win over requested speed. More speed should be validated from fresh logs, not
assumed from a higher nominal constant.

`R` ramps to ready thrust, `T` engages the 3 ft helper, and `F` starts the
figure-8 once the vehicle is settled. A second `F` starts the controlled
return-to-start and landing sequence.

`FIGURE8_RADIUS_X_M` and `FIGURE8_RADIUS_Y_M` describe the requested path, not
a guaranteed physical size. Before flight, the script reduces that request to
fit the configured cage bounds and tracking reserve. The active fast-path
planner uses the older raw-corner model and plans a compact horizontal route of
about `5.4 m x 5.4 m`.

## Keyboard Controls

| Key | Action |
| --- | --- |
| `R` | Ramp to the ready-thrust target. |
| `T` | Toggle the pre-figure-8 3 ft height helper. |
| Up / Down | Before height hold: change raw thrust. During 3 ft hold or figure-8: nudge the height target. |
| PageUp | Larger upward thrust or height-target nudge. |
| PageDown | Normal descent ramp. |
| `F` | Start figure-8; when active, return to the figure-8 start and land. |
| `H` | Lock the current X/Y hold target. |
| `A` / `D` | Roll trim. |
| `W` / `S` | Pitch trim. |
| `J` / `L` | Yaw-target trim. |
| `C` | Clear attitude and yaw trims. |
| Space | Immediate zero-thrust command. |
| `Q` / Esc | Request safety descent and exit after landing. |

## What the Script Uses From OptiTrack

The VRPN rigid-body pose is transformed into the local flight frame as:

```text
local X = -raw Y
local Y =  raw X
local Z =  raw Z
```

It logs the transformed position (`mocap_x`, `mocap_y`, `mocap_z`), raw rigid
body orientation as a normalized quaternion, pose age, frame count, controller
commands, safety state, and target/error data. The controller uses the position
and yaw derived from the quaternion; it does not send VRPN data directly to the
Crazyflie's estimator in this manual-thrust workflow.

## Boundaries and Safety

The active safety values are deliberately all visible at the top of the flight
script. In particular, cage bounds, stale-mocap handling, climb-rate protection,
and emergency zero-thrust behavior remain active. The hard height limits are
currently disabled by `ENFORCE_HEIGHT_LIMITS = False`; that is an intentional
experiment setting, not a claim that high-altitude flight is intrinsically safe.

`MOCAP_STALE_RESUME_FIGURE8_S` permits brief coverage gaps to recover without
pausing the figure-8 timeline. A prolonged stale interval levels the commands
and eventually initiates the configured safety descent.

The safety behavior is intentionally asymmetric:

- `Space` is the operator's deliberate emergency zero-thrust command.
- `Q` or Esc begins the same controlled safety-descent path used by automatic
  guards, rather than issuing a hard cut.
- Target-error, persistent cage-boundary, excessive climb-rate, stale-mocap,
  and unexpected host-exit paths request a neutral, controlled descent when
  fresh mocap is available. The cleanup path also attempts a neutral
  mocap-guided descent when the host exits while thrust is nonzero.

One hard-fall investigation in
`mocap-assisted-figure8-20260720-092217.csv` found the script still logging
roughly `34k..37k` commanded thrust while the measured height fell rapidly;
the battery voltage simultaneously sagged as low as about `2.28 V`. That is
consistent with a power/device-side interruption, not evidence of an
intentional script zero-thrust command. This is an inference from the log, not
a definitive hardware diagnosis. Battery enforcement is currently disabled in
the script at the operator's request, so pack condition remains a manual
preflight responsibility.

To suppress derivative spikes from a bad mocap frame, measured velocities are
plausibility-clamped at `5.0 m/s` horizontally and `2.0 m/s` vertically before
they enter the filtered controller state. This does not replace stale-mocap
handling; it protects the controller from a single fresh-but-jumpy sample.

## Measured Horizontal Coverage

The cage is rectangular and OptiTrack coverage is asymmetric. The numbers
below are conservative **route caps** measured by the edge-probe workflow at
about 3 ft above takeoff. They are not physical wall coordinates and are not a
license to fly to the visual edge of the cage: obstacle clearance and tracking
reserve are already folded into the values.

| Relative direction | Current cap | Most useful evidence |
| --- | ---: | --- |
| Forward | `5.0 m` | `mocap-cage-edge-probe-20260717-094258.csv` reached `4.95 m` fresh. |
| Backward | `4.5 m` | `mocap-cage-edge-probe-20260717-094258.csv` reached `4.44 m` fresh. |
| Left | `3.0 m` | `mocap-cage-edge-probe-20260717-104848.csv` reached `2.86 m` and returned fresh. |
| Right | `3.5 m` | `mocap-cage-edge-probe-20260717-104848.csv` reached `3.41 m` and returned fresh. |

The right cap deliberately stops short of a previous rightward stale event at
about `3.92 m`. The left cap also stays below earlier coverage trouble farther
out. Use this asymmetric envelope when choosing path size or creating another
path script; do not assume a symmetric square is safe just because the physical
cage looks large enough.

Because the cage is treated as rectangular, the four values also define an
inferred, start-relative corner rectangle that can be evaluated for future
figure-8 planning:

```text
front-left  = (+5.0 m, +3.0 m)
front-right = (+5.0 m, -3.5 m)
back-right  = (-4.5 m, -3.5 m)
back-left   = (-4.5 m, +3.0 m)
```

The larger inferred rectangle was feasible in
`flight_logs/mocap-assisted-figure8-20260720-144526.csv`: the planned path was
`8.4 m x 5.4 m`, completed without a safety descent, and remained mocap-fresh.
It is a successful feasibility result, not the active route configuration.

For the current fast-path work, `mocap_manual_thrust_assisted_figure8.py`
uses the older raw-Motive corner model to plan the compact route. The measured
corridor remains active for safety checks and path-speed governing. These
limits are triangulated from independent axis probes, not direct diagonal
corner measurements, so retain stale-mocap and path-error safety guards.

The successful 2026-07-17 lateral probe had no stale samples or safety descent,
and maintained its outbound altitude within roughly `0.89..1.06 m` above
takeoff. Its return-to-center behavior is intentionally sharper than the
outbound leg, with brief lateral speed estimates near `3 m/s`. That profile is
accepted only as the presently tested edge-probe return; new paths should keep
their own normal path motion slower and should not increase the return speed
without reviewing a new log.

### Corner Survey Follow-up

`mocap_cage_corner_probe.py` is a separate corner-coverage survey tool built
from the successful edge-probe control stack. After the normal `R`, low-hover,
and settled `T` sequence, `F` runs these center-origin routes:

```text
front-left -> front-right -> back-left -> back-right
```

For each route it travels at the proven speed to the applicable known
forward/backward corridor cap, then approaches the calculated perpendicular
limit. It holds there for `1.5 s` of settled fresh mocap, records the measured
local point, and returns to center before the next route. Brief mocap gaps
pause and then resume the same stage; only a continuous `3 s` stale interval
records the last fresh local coordinate as a coverage boundary and uses the
existing no-blind-return safety behavior. The script never deliberately flies
beyond the calculated rectangle. Review its CSV and path image before copying
any measured corner into a flight-path configuration.

The first front-left corner attempt on 2026-07-20 found a sustained OptiTrack
dropout at about `4.72 m` forward and `2.15 m` left. The corner-probe script
therefore temporarily uses a front-left lateral limit of `1.85 m` while keeping
the temporary one-foot forward inset (`4.6952 m` forward). This is a
corner-specific optical-coverage margin, not a revised physical cage size.

The following run, `mocap-cage-corner-probe-20260720-103251.csv`, reached and
held the front-left point for `1.5 s` of fresh mocap near `4.73 m` forward and
`1.80 m` left, then returned normally. Its front-right route went continuously
stale near `4.64 m` forward and `0.69 m` right, before it reached the inferred
rectangle intersection. The script now uses a provisional front-right
rightward limit of `0.40 m`, retaining roughly `0.29 m` of observed coverage
margin for the next multi-corner survey. This difference is optical coverage
at a diagonal, not evidence that the independent forward or right corridor is
wrong.

The later receive-only hand survey `mocap-corner-coverage.csv` directly
measured all four diagonal boundaries at one stable chair-height plane. The
last fresh local points before sustained dropout were front-right
`(+3.790, -3.072)`, back-right `(-3.687, -2.829)`, back-left
`(-3.967, +2.496)`, and front-left `(+3.598, +2.538)` metres. These are the
best direct optical-coverage measurements so far, but they are not automatic
flight waypoints: reserve at least `0.30 m` radially and validate at the
actual flight height and yaw before incorporating them into a path.

## Height Coverage Finding

The retired height-probe experiment
`flight_logs/mocap-height-map-20260715-155351.csv` is the current best evidence
for vertical mocap coverage near the cage center. It reached a fresh mocap
height of about `3.996 m` above the recorded start point (`mocap_z ~= 4.035 m`)
with only about `0.06 m` horizontal drift.

Coverage near that height should be treated as intermittent, not guaranteed.
The first stale interval began around `3.991 m` above start
(`mocap_z ~= 4.030 m`), and several stale/reacquire cycles happened near the
same ceiling region. Earlier height-probe logs stayed fresh through about
`1.76 m` above start, so the normal 3 ft figure-8 target (`0.9144 m`) is well
inside the proven coverage volume.

The experimental `mocap_height_mapper.py` script has been removed. Keep using
the manual path scripts plus their CSV logs for flight work; keep stale-mocap
guards enabled even below the measured upper coverage edge.

## Repository Notes

`AIMSLAB_AUTONOMY_RUNBOOK.md` and `HANDOFF.md` preserve earlier high-level
commander and calibration work. They are useful background, but this document
and the constants in the manual-thrust script describe the current flight
baseline.
