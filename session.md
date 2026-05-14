# Session Notes

Date: 2026-05-12
Repo: `/home/alwin-raj/Desktop/drone/crazyflie-clients-python`
Branch: `aimslab/work`

## Goal

Work through the local Crazyflie client repo, review recent `alwnraj` changes, get the repo running, validate controller input, and diagnose mocap flight behavior.

## Commit Review

Reviewed reachable commits by author `alwnraj`.

Found one reachable commit:

- `e801be6` - `Fix controller safety and mocap trajectory startup`

Review findings:

1. High: `mocap-extpose-figure8.py` now skips the pre-takeoff relocation to the computed safe launch point and instead takes off from the current position before later safety checks.
2. Medium: `test_flight_with_controller.py` now waits for low thrust after arming, which can leave the Crazyflie armed indefinitely if the thrust axis is never detected or mapped wrong.
3. Medium: controller autodiscovery now picks the first `/dev/input/js*` device without verifying it is actually the Logitech F310.

## Repo Run Path

Main app / GUI:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
cfclient
```

Alternative GUI launch:

```bash
python3 -m cfclient.gui
```

Custom controller workflow in this repo:

```bash
./RUN_THIS_FIRST.sh
python3 demo_controller_live.py
python3 test_flight_with_controller.py
```

## Radio / Connectivity Debugging

Initial mocap example attempts:

```bash
python3 src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-example.py
```

Observed issues:

- `Too many packets lost`
- `Resource busy`

Interpretation:

- `Too many packets lost` happened while opening the Crazyflie radio link, before mocap logic.
- `Resource busy` indicated the Crazyradio dongle was already held by another process, likely `cfclient` or a previous crashed script.

Later state:

- radio link became stable
- mocap example connected
- VRPN saw rigid body `crazyflie_21`
- estimator converged successfully

Warnings observed during successful connection:

- VRPN minor version mismatch warning
- `platform.send_arming_request` deprecation warning
- Crazyflie reported CRTP protocol version `9`, causing legacy fallback
- VRPN cleanup warning on shutdown

## Controller Detection Debugging

Problem:

- Logitech controller appeared not to work with `test_logitech_controller.py`

Initial discovery:

```bash
ls -la /dev/input/js*
```

showed:

- `/dev/input/js0`

The test script read `/dev/input/js0` but showed:

- empty device name
- `Axes: 2, Buttons: 2`
- no useful button/stick response

Device identification:

`/dev/input/js0` turned out to be:

- `Melfas LGD AIT Touch Controller Mouse`

So the script was reading the wrong joystick-class device, not the Logitech pad.

Root cause found by user:

- the Logitech controller was plugged into a dead USB port

After moving to a working port:

- the controller worked correctly

## Manual Controller Flight Validation

The direct controller flight path worked through:

```bash
python3 test_flight_with_controller.py
```

Key conclusion:

- low-level command streaming via `cf.commander.send_setpoint(...)` works on this setup

Observed behavior from log output:

- roll, pitch, yaw, and thrust all responded
- cleanup and disarm worked

Important interpretation:

- the printed `Alt: ...` values in `test_flight_with_controller.py` are not trustworthy as real altitude truth for flight validation
- the useful signal from that test was that manual command channels were reaching the drone correctly

Manual flight tuning learned by user:

- about `57%` thrust is the practical hover / liftoff sweet spot
- pitch trim of about `1.8` helps keep the drone stable

## Mocap High-Level Commander Diagnosis

Problem:

- `mocap-map-boundaries.py` armed and printed takeoff messages, but the drone did not actually rise
- reported position stayed near ground level, around `z ~= 0.03`

Root cause identified:

- controller/manual script uses low-level `send_setpoint(...)`
- mocap scripts use high-level commander functions like:
  - `takeoff()`
  - `go_to()`
  - `start_trajectory()`
- GUI code in `src/cfclient/ui/tabs/FlightTab.py` enables `commander.enHighLevel = 1` before using takeoff
- mocap scripts were not enabling `commander.enHighLevel`

Conclusion:

- on this firmware/setup, high-level motion commands were being ignored until `commander.enHighLevel` was enabled

## Code Changes Made

Made the minimal possible change: enabled the high-level commander in these mocap scripts before issuing high-level commands:

- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-example.py`
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-map-boundaries.py`
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-boundary-aware.py`
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-figure8.py`

Nature of change:

- added helper `enable_high_level_commander(cf)`
- called `cf.param.set_value('commander.enHighLevel', '1')`
- no other flight logic was intentionally changed

## Result After High-Level Enablement

Re-tested:

```bash
python3 src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-example.py
```

Result:

- drone took off quickly and crashed

Interpretation:

- previous "not moving" issue is fixed
- high-level commander is now actively executing commands
- current failure is flight behavior / state estimation / frame alignment, not command delivery

## Mocap / Pose Data Observations

User-reported pose at takeoff on ground:

- Quaternions: `0.282, -0.015, 0.09, 0.959`
- Position: `-0.107 -0.941 0.033`

User-reported pose shortly after takeoff in mid-air:

- Quaternions: `-0.789, -0.005, -0.012, 0.614`
- Position: `-0.369 0.168 0.091`

Important interpretation:

- a small vertical takeoff should mostly increase `z`
- instead, `y` changed by about `+1.11 m`
- `x` changed by about `-0.26 m`
- `z` changed only from about `0.033` to `0.091`

This strongly suggests frame / pose interpretation problems, such as:

- rigid body origin offset
- wrong quaternion / axis convention from VRPN
- mismatch between mocap world frame and estimator expectations
- bad yaw / orientation alignment causing lateral correction during takeoff

## Later Calibration Findings

User later established a more reliable cage-center reference point:

- cage center reports approximately `0.000 0.000 0.037`
- first reported horizontal value corresponds to `y`
- second reported horizontal value corresponds to `x`
- the reported floor baseline for `z` is about `0.037`

Working interpretation:

- the cage center is the mocap origin in `x/y`
- `z = 0.037` is effectively the floor-level baseline for this rigid body setup
- practical height above floor should be treated as:
  - `height_above_floor = reported_z - 0.037`

Examples:

- reported `z = 0.037` -> on the floor
- reported `z = 0.137` -> about `10 cm` above the floor
- reported `z = 0.837` -> about `80 cm` above the floor

This makes the `z` offset itself unsurprising; the larger remaining concern is orientation alignment.

## Orientation Validation Attempts

The user next suspected that `x/y` might actually be fine and that orientation was the real issue.

### First orientation check

User held the drone at a maintained height and reported:

- rightside up:
  - quaternion: `0.989 0.039 0.003 -0.140`
- upside-down:
  - quaternion: `0.543 0.835 0.050 -0.065`

Important caveat:

- upside-down readings were not continuous because the OptiTrack cameras are above the cage
- this was not a useful yaw-alignment test because flipping upside down mixes roll/pitch and visibility issues

### Flat yaw-style orientation checks

The next recommendation was to keep the drone level and rotate it in 90 degree steps:

- nose forward
- nose right
- nose backward
- nose left

First reported set:

- nose forward:
  - quaternion: `0.806 -0.068 -0.046 0.586`
  - position: `0.014 -0.019 0.146`
- nose right:
  - quaternion: `0.995 0.002 0.005 -0.099`
  - position: `0.035 -0.049 0.150`
- nose backward:
  - quaternion: `0.688 -0.182 0.727 -0.012`
  - position: `0.015 -0.037 0.133`
- nose left:
  - quaternion: `0.565 0.478 0.572 0.346`
  - position: `0.040 0.002 0.138`

Issue with that set:

- the user noted the backward sample was actually "nose up"
- that means pitch contaminated the test, so it did not isolate yaw cleanly

Second reported set, again with quaternions from OptiTrack:

- nose forward:
  - quaternion: `0.776 -0.020 0.025 0.630`
  - position: `0.029 -0.017 0.170`
- nose right:
  - quaternion: `0.999 -0.020 -0.008 -0.023`
  - position: `0.034 -0.032 0.152`
- nose backward:
  - quaternion: `0.760 0.065 -0.644 -0.065`
  - position: `0.013 -0.021 0.170`
- nose left:
  - quaternion: `0.567 0.555 -0.375 0.480`
  - position: `0.036 -0.016 0.149`

Issue with that set:

- the user noted the backward case was actually "nose facing down"
- again, that means the test was not a clean level-only yaw sweep

What these orientation tests do show:

- position stayed relatively stable during manual holding/rotation
- horizontal position changes were only a few centimeters
- `z` stayed in a narrow band

Current interpretation:

- position tracking looks much more credible than it did during the autonomous crash
- orientation tracking is changing, but has not yet been validated in a clean way
- the autonomous takeoff/crash could still be caused by bad attitude alignment, wrong forward-direction assumptions, or quaternion/frame convention mismatch

## Current Best Understanding

What is confirmed working:

- repo builds and runs in the local venv
- Crazyradio link works
- mocap feed connects
- rigid body `crazyflie_21` is seen
- Kalman estimator can converge
- Logitech controller works
- low-level manual control works
- cage center / floor baseline are partially calibrated:
  - `y ~= 0.000`, `x ~= 0.000`, `z ~= 0.037` at cage center on the floor

What is still not trustworthy:

- autonomous mocap takeoff
- high-level trajectory flight
- mocap pose / orientation alignment for autonomous stabilization
- OptiTrack quaternion interpretation for level yaw orientation

## Recommended Next Steps

1. Do not run autonomous mocap trajectory scripts again yet:
   - `mocap-extpose-example.py`
   - `mocap-extpose-figure8.py`

2. Validate mocap pose with props off:
   - stream pose continuously
   - move the drone by hand along one axis at a time
   - verify `x`, `y`, and `z` change in expected directions
   - rotate yaw by hand and verify orientation changes cleanly while the drone stays level
   - avoid upside-down or pitched tests; they do not isolate yaw

3. Do a conservative props-on manual test:
   - use controller only
   - use the known hover thrust around `57%`
   - use pitch trim around `1.8`
   - verify whether mocap `z` rises cleanly without large `x/y` jumps

4. Repeat the orientation check on a flat surface if possible:
   - keep the drone level
   - record four orientations in 90 degree steps
   - confirm position stays nearly fixed while the quaternion changes

5. Inspect and correct the mocap pose mapping before further autonomous tests.

6. After frame alignment is trustworthy, retry a minimal autonomous action:
   - takeoff and land only
   - no trajectory upload
   - no figure-8

7. Longer term:
   - update Crazyflie firmware to reduce protocol/deprecation mismatch risk

## Current Working Tree

Modified files:

- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-example.py`
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-map-boundaries.py`
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-boundary-aware.py`
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-extpose-figure8.py`

Added file:

- `session.md`

## 2026-05-13 Update: Guarded Mocap Takeoff Script

User request:

- add the next-step script for a safe mocap-driven takeoff test
- update this session file so a future agent has context
- commit and push the change to GitHub

Reason for this step:

- Motive/VRPN pose is now confirmed to stream successfully
- high-level commander enablement made the drone respond to autonomous commands
- the autonomous response was not yet safe; the drone took off quickly and crashed
- therefore the next script should not run trajectories or horizontal paths

Added script:

- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guarded-takeoff.py`

Purpose:

- connect to the Crazyflie over Crazyradio
- connect to Motive/VRPN using:
  - `host_name = '192.168.1.42:3883'`
  - `mocap_system_type = 'vrpn'`
  - `rigid_body_name = 'crazyflie_21'`
- stream full external pose into the Crazyflie with `cf.extpos.send_extpose(...)`
- require fresh mocap pose before arming
- require the pose to be stable for a short window before takeoff
- enable the Kalman estimator and high-level commander
- reset the estimator while external pose is flowing
- log `stateEstimate.x/y/z` and print it beside the mocap position
- require the operator to press ENTER before arming
- take off only to a low target:
  - floor baseline: `FLOOR_Z = 0.037`
  - height above floor: `TAKEOFF_HEIGHT_ABOVE_FLOOR = 0.15`
  - command target: `TAKEOFF_Z = 0.187`
- hover briefly
- land and disarm

Safety guards in the script:

- aborts if mocap pose is stale
- aborts if start pose is outside the configured cage bounds
- aborts if pose is not stable enough before takeoff
- aborts and lands if live pose leaves bounds during takeoff/hover
- aborts and lands if horizontal drift exceeds `MAX_HORIZONTAL_DRIFT = 0.35`
- does not upload trajectories
- does not command horizontal motion

Current default cage bounds in the script:

```python
CAGE_BOUNDS = {
    'x_min': -1.5,
    'x_max': 1.5,
    'y_min': -1.5,
    'y_max': 1.5,
    'z_min': 0.0,
    'z_max': 2.0,
}
SAFETY_MARGIN = 0.20
```

These are intentionally conservative placeholders. They should be updated with the measured cage bounds once frame alignment and low takeoff are trustworthy.

Recommended run command:

```bash
python3 src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guarded-takeoff.py
```

Recommended physical setup before running:

- Motive is running
- rigid body `crazyflie_21` is visible
- Crazyradio is not held by `cfclient` or another script
- drone is near cage center
- props on only when ready for the low takeoff test
- operator has a physical emergency stop / power-off option ready

How to interpret the output:

- mocap and `stateEstimate` positions should be close after estimator reset
- during takeoff, `z` should rise clearly
- `x/y` should not jump or drift significantly
- if the script lands due to stale pose, boundary violation, or drift, do not proceed to trajectory scripts

Next step after this script succeeds:

1. repeat the guarded takeoff a few times from cage center
2. reduce or explain any mismatch between mocap position and `stateEstimate`
3. add a guarded small horizontal move script:
   - take off low
   - move only `10 cm` along one axis
   - return to start
   - land
4. only after that, revisit boundary mapping or figure-8 trajectories

## 2026-05-13 Update: Guarded Raw-Thrust Test Script

User asked whether thrust could be controlled directly while keeping the mocap
guards and estimator setup the same.

Added script:

- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guarded-thrust-test.py`

Purpose:

- keep the same Motive/VRPN pose streaming path as `mocap-guarded-takeoff.py`
- keep the Kalman estimator reset and `stateEstimate.x/y/z` comparison logging
- keep the fresh-pose, stable-pose, cage-boundary, stale-pose, and horizontal
  drift guards
- replace high-level `takeoff(...)` with low-level
  `cf.commander.send_setpoint(roll, pitch, yawrate, thrust)`
- command zero roll, zero pitch, and zero yaw rate while ramping raw thrust

Default behavior:

- target height is intentionally lower than the high-level takeoff script:
  - `TARGET_HEIGHT_ABOVE_FLOOR = 0.12`
  - `TARGET_Z = 0.157`
- raw thrust ramp starts at `START_THRUST = 20000`
- ramp ceiling defaults to `MAX_THRUST = 34000`
- thrust increments by `THRUST_STEP = 400` every `RAMP_INTERVAL = 0.08s`
- the script cuts thrust if the target height is reached, if any guard trips,
  or if the operator interrupts it

Recommended run command:

```bash
python3 src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guarded-thrust-test.py
```

Important safety note:

- this is not altitude hold; it is a raw thrust ramp with mocap guardrails
- if it does not lift by `MAX_THRUST`, the script aborts instead of continuing
  into the user's observed hover/liftoff range
- tune `MAX_THRUST` upward only after confirming the ramp behavior is stable

Review / verification before commit:

- `mocap-guarded-thrust-test.py` was reviewed as an uncommitted change
- no actionable review findings were found
- syntax check passed with `python3 -m py_compile`
- whitespace check passed with `git diff --check`

## 2026-05-14 Update: Motor Lockout Diagnosis

Problem observed:

- `mocap-guarded-thrust-test.py` connected to Crazyflie and OptiTrack/VRPN
- mocap pose was fresh and stable
- the script armed, counted down, and ramped raw thrust up to `39000`
- mocap `z` stayed at the floor baseline around `0.037m`
- motors did not spin

Diagnostic changes made to the guarded thrust script:

- changed the default URI to match the known-working controller path:
  - `radio://0/80/2M`
- added a 3 second command-line preflight countdown
- added pitch trim from manual testing:
  - `PITCH_DEG = 1.8`
- raised the raw thrust test cap to the manually observed low-liftoff range:
  - `MAX_THRUST = 39000`
- added a `manual_percent` control mode for comparing against GUI-style
  percentage thrust
- temporarily disabled mocap feeding and Kalman setup:
  - `FEED_MOCAP_TO_CRAZYFLIE = False`
  - `USE_KALMAN_ESTIMATOR = False`
- kept mocap active as an external safety monitor for pose, bounds, stale data,
  and drift checks

Root cause found:

- the Crazyflie was locked out
- `cfclient` showed the locked state
- rebooting the Crazyflie cleared the lockout
- after reboot, the drone/motors worked again

Current conclusion:

- the repeated "motors did not move" result was most likely caused by the
  Crazyflie lockout state, not by the low-level `send_setpoint(...)` command
  path itself
- before interpreting script failures, verify in `cfclient` that the Crazyflie
  is not locked and that motors respond to manual thrust

Recommended next test:

1. close `cfclient` so the Python script has exclusive Crazyradio access
2. reboot the Crazyflie
3. place the drone at cage center with OptiTrack tracking `crazyflie_21`
4. run:

```bash
python3 src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guarded-thrust-test.py
```

Expected behavior:

- motors should begin responding during the thrust ramp
- mocap `z` should rise from around `0.037m`
- the script should cut thrust once `TARGET_Z = 0.157m` is reached
- press `Ctrl+C` immediately if the drone moves laterally, rises too fast, or
  looks unstable

## 2026-05-14 Update: Manual Flight Logging Plan

User asked if a logging script could record a GUI-assisted Logitech controller
test flight so the results can be interpreted later and used to improve the
autonomous scripts.

Important constraint:

- when `cfclient` is connected and controlling the drone through Crazyradio, a
  second Python process should not also try to own the Crazyradio link
- therefore the logger should not command the drone or connect to Crazyflie by
  default during GUI-assisted flight

Added script:

- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-controller-flight-logger.py`

Purpose:

- read Logitech F310 joystick events from `/dev/input/js1` by default
- connect to Motive/VRPN at `192.168.1.42:3883`
- track rigid body `crazyflie_21`
- write a CSV log under `flight_logs/`
- record controller command values alongside mocap position, quaternion, derived
  height above floor, derived mocap velocity, and horizontal distance from the
  start position

Recommended workflow:

1. start Motive and confirm rigid body `crazyflie_21` is visible
2. start `cfclient` and connect/control the drone with the Logitech controller
3. in a second terminal, run:

```bash
python3 src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-controller-flight-logger.py
```

4. perform a short manual test flight:
   - take off gently
   - hold a low hover
   - make very small pitch/roll/yaw inputs one at a time
   - land
5. stop the logger with `Ctrl+C`
6. provide the generated CSV file for analysis

What the CSV can help estimate:

- actual liftoff thrust range
- approximate hover thrust range
- vertical response delay from thrust changes
- horizontal drift during nominal hover
- whether pitch/roll commands correlate with the expected mocap x/y motion
- whether yaw inputs create unexpected translation or frame-alignment symptoms
