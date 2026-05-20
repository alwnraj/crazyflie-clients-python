# AIMSLab Crazyflie Mocap Autonomy Runbook

## Goal

Get to repeatable autonomous indoor figure-8 flight by using the standard Crazyflie
mocap stack:

```text
OptiTrack/Motive -> VRPN -> laptop Python script -> cf.extpos -> Kalman estimator -> high-level position commands
```

The immediate goal is not figure-8. The immediate goal is repeatable mocap-backed
hover. Figure-8 should only happen after validate, hover, steps, and circle all
pass.

## Main Script

Use `mocap_autonomy_ladder.py` as the canonical operator script.

Do not add a new flight script for every experiment. If a new safety check or
flight mode is needed, add it to the ladder script.

Keep these older scripts in their current roles:

- `mocap_vertical_thrust_mapper.py`: diagnostic/manual thrust mapping only.
- `mocap-guarded-takeoff.py`: historical reference for extpose/HLC setup.
- `mocap-guarded-thrust-test.py`: historical reference for estimator-vs-mocap gates.
- `mocap-extpose-figure8.py`: do not use for flight yet. It jumps too far ahead.

## Preflight Checklist

Before every powered run:

- Crazyflie battery is above `3.75V`.
- Battery is secured and not sagging under the mocap deck or marker mount.
- Props are correct and fully seated.
- Guards are installed and not touching props.
- Cage is clear.
- Motive is tracking rigid body `crazyflie_21`.
- VRPN stream is reachable at `192.168.1.42:3883`.
- Crazyradio is connected.
- `cfclient` or any other competing Crazyradio process is closed.
- You have a physical power cutoff plan.
- First run of the day is `validate`, not `hover`.

If the script fails the cage-bounds gate, update the measured bounds with
`--x-min`, `--x-max`, `--y-min`, and `--y-max`. Do not bypass the bounds check
just to make a run start.

## Command Ladder

Run commands on the Linux ThinkPad, from the machine that has `cflib`,
`motioncapture`, Crazyradio access, and VRPN access.

### 1. Validate

No autonomous flight. The script streams mocap into the Crazyflie estimator,
resets Kalman, then logs estimator-vs-mocap agreement while the drone is moved
by hand.

```bash
python3 mocap_autonomy_ladder.py validate
```

Pass criteria:

- Rigid body is found.
- Mocap pose stays fresh.
- Kalman estimator follows mocap.
- Max estimator error stays below `0.08m..0.10m`.
- No bounds or stale-pose guard trips.

Do not fly if validate fails.

### 2. Hover

Autonomous takeoff to `0.35m` above the start position, hold for `15s`, then
land.

```bash
python3 mocap_autonomy_ladder.py hover
```

Pass criteria:

- Three successful hover runs in a row.
- No stale mocap.
- No estimator disagreement.
- Max horizontal drift stays below `0.15m`.
- Land is controlled.

If hover is unstable, do not continue to steps. Re-run validate, inspect logs,
and check marker tracking/orientation.

### 3. Steps

Small position moves only: `+x`, `-x`, `+y`, `-y`, returning to center each time.

```bash
python3 mocap_autonomy_ladder.py steps
```

Pass criteria:

- Each step is only `0.10m`.
- The drone returns to center.
- Max tracking error stays below `0.20m`.
- No growing estimator error.

### 4. Circle

Small, slow circle around the start position.

```bash
python3 mocap_autonomy_ladder.py circle
```

Pass criteria:

- Radius is `0.05m`.
- Period is `24s`.
- No guard trips.
- Error does not grow over the path.

### 5. Figure-8

Only attempt after validate, three hovers, steps, and circle pass.

```bash
python3 mocap_autonomy_ladder.py figure8
```

Initial pass criteria:

- Radius is tiny: `0.05m..0.08m`.
- Height is `0.35m`.
- Period is `24s`.
- One full period completes.
- Max tracking error stays below `0.20m`.

## Useful Options

Defaults are intentionally conservative:

```bash
python3 mocap_autonomy_ladder.py validate \
  --uri radio://0/80/2M \
  --host 192.168.1.42:3883 \
  --body crazyflie_21 \
  --height 0.35 \
  --pose-stale-timeout 0.30 \
  --estimate-tolerance 0.08 \
  --max-radius-from-start 0.45 \
  --max-height-above-start 0.60 \
  --min-battery 3.75 \
  --pose-mode extpose
```

If yaw/quaternion alignment looks wrong in validate, try position-only injection:

```bash
python3 mocap_autonomy_ladder.py validate --pose-mode extpos
```

Use `extpos` only as a diagnostic step. The preferred path is `extpose` once the
rigid body orientation is trusted.

## Logs

Each run writes a CSV under `flight_logs/`.

Review these columns first:

- `stop_reason`
- `mocap_age_s`
- `estimate_error_m`
- `battery_v`
- `height_above_start_m`
- `radius_from_start_m`
- `target_error_m`
- `guard_ok`

Every failed run should have a stop reason. If a run fails without a useful stop
reason, improve the script before doing more flight tests.

## What Not To Run Yet

Do not use `mocap-extpose-figure8.py` for flight yet.

Do not tune manual thrust into a figure-8 controller.

Do not continue to the next milestone after a guard trip. Treat guard trips as
data, inspect the CSV, and repeat the previous milestone.

Do not increase radius, height, or speed in the same run. Change one variable at
a time.

## Static Checks

Run these before committing changes:

```bash
PYTHONPYCACHEPREFIX=/tmp/crazyflie-pycache python3 -m py_compile \
  mocap_autonomy_ladder.py \
  keyboard_thrust_test.py \
  mocap_vertical_thrust_mapper.py \
  test_mocap_autonomy_ladder.py
```

```bash
PYTHONPYCACHEPREFIX=/tmp/crazyflie-pycache python3 -m unittest \
  test_mocap_autonomy_ladder.py
```

The unit tests are pure Python and do not need `cflib`, `motioncapture`, VRPN, or
Crazyradio hardware.
