# Crazyflie Mocap Flight Handoff

Date: 2026-05-19
Repo: `/home/alwin-raj/Desktop/drone/crazyflie-clients-python`
Branch: `aimslab/work`

## Current Goal

Develop toward autonomous flight inside the OptiTrack cage. The current safe
path is low-level manual thrust plus mocap-assisted horizontal hold. Do not jump
back to high-level trajectory takeoff until hold behavior is repeatable.

## Hardware / Environment

- Crazyflie 2.1 Brushless.
- Firmware boot log showed `2025.02`, brushless motor driver active.
- No positioning deck.
- Crazyradio URI: `radio://0/80/2M`.
- OptiTrack/VRPN host: `192.168.1.42:3883`.
- Rigid body: `crazyflie_21`.
- Floor/cage-center mocap baseline is approximately `z = 0.037`.
- User found first horizontal coordinate maps to physical `y`, second to
  physical `x`, and third to height `z`.
- Close `cfclient` before running Python control scripts; only one process can
  own Crazyradio.
- `Ctrl+C`, `q`, Space, and script cleanup are software stops only. Keep a
  physical power-off option ready.

## Important History

- Logitech controller initially looked broken because the USB port was dead.
  It worked after moving ports.
- A joystick device at `/dev/input/js0` was the touchscreen mouse, not the
  controller.
- High-level commander commands were ignored until scripts set
  `commander.enHighLevel = 1`.
- After enabling high-level commander, `mocap-extpose-example.py` took off too
  aggressively and crashed. Avoid high-level autonomous scripts for now.
- Raw motor tests initially showed no motor movement because the Crazyflie was
  locked in the GUI; rebooting cleared it.
- A later no-lift issue was caused by props installed incorrectly. After fixing
  the props, the drone generated thrust and flew.
- User reports around `57%` GUI thrust is a practical liftoff/hover area.
- User reports above about `60%` thrust it climbs quickly.

## Active Script

Use:

```bash
python3 mocap_vertical_thrust_mapper.py
```

Modes:

- `guard-only`: keyboard thrust, zero roll/pitch, mocap logging and guards.
- `hold-xy`: keyboard thrust, mocap-based roll/pitch correction.
- `figure8`: keyboard thrust, moving horizontal target after airborne trigger.

Controls:

- Up / Down: thrust +/- small step.
- PageUp / PageDown: thrust +/- big step.
- Space: immediate thrust cut to zero.
- `q` / Esc: immediate cut, disarm, exit.
- Normal landing should use PageDown, not `q`.

The script logs detailed CSVs to `flight_logs/`.

## Current Script Behavior

`mocap_vertical_thrust_mapper.py` currently:

- reads mocap pose continuously from VRPN
- logs every mocap frame
- logs raw thrust, thrust percent, battery, estimator z, mocap pose, yaw,
  horizontal drift, target error, velocity, body-frame error, and commanded
  roll/pitch
- caps keyboard thrust with `--max-commanded-thrust`
- has an altitude ceiling with `--max-height-above-start`
- delays XY correction until `--control-activation-height`
- resets the XY target to the current airborne position when XY control first
  activates
- keeps drift guard active before XY activation, so low-altitude sliding or
  mocap jumps still abort the run

Current default XY signs:

- `roll_sign = +1`
- `pitch_sign = -1`

`pitch_sign=-1` has looked better than `pitch_sign=+1` in recent tests.

## Latest Useful Command

Run this next, preferably with a fresh battery:

```bash
python3 mocap_vertical_thrust_mapper.py \
  --mode hold-xy \
  --kp-xy 12.0 \
  --kd-xy 6.0 \
  --max-angle-deg 10.0 \
  --pitch-sign -1.0 \
  --roll-sign 1.0 \
  --control-activation-height 0.03 \
  --max-horizontal-drift 0.60 \
  --max-target-error 0.60 \
  --max-height-above-start 0.35 \
  --max-commanded-thrust 36000 \
  --step 250 \
  --big-step 500
```

Operator guidance:

- Use PageUp until the drone lifts cleanly.
- Stop increasing thrust once airborne.
- Use PageDown to descend.
- Do not use `q` for normal landing; it cuts immediately.

## Latest Log Read

Latest analyzed log:

- `flight_logs/mocap-vertical-thrust-map-20260519-131035.csv`

Summary:

- rows: `336`
- duration: `29.97s`
- start z: `0.037`
- max z: `0.123`
- max height above start: `0.086m`
- XY control activated at `14.67s`
- active rows: `175`
- final/max thrust: `35000`
- final/max drift: `0.598m`
- max target error: `0.535m`
- battery sagged to about `3.65V`

Interpretation:

- The activation gate worked.
- The drone did not get enough clean altitude for a stable hold.
- This was still mostly a low-altitude / underpowered hold run.
- The next run should use a fresh battery and a slightly higher thrust cap
  (`36000`) while keeping the `0.35m` height ceiling.

## Log Analysis Checklist

When a new CSV appears under `flight_logs/`, summarize:

- row count and duration
- start, max, and final `mocap_z`
- max height above start
- first time above `+0.03m` and `+0.05m`
- max/final thrust
- min/final battery
- max/final horizontal drift
- max/final target error
- whether `xy_control_active` ever became `1`
- first active row
- max roll/pitch command
- final velocity direction
- whether the trip was height, drift, target error, stale mocap, or operator
  exit

Useful quick command pattern:

```bash
python3 - <<'PY'
import csv, math
path = 'flight_logs/REPLACE_WITH_LOG.csv'
rows = list(csv.DictReader(open(path, newline='')))
def fl(r, k, d=float('nan')):
    try:
        return float(r.get(k, '') or d)
    except Exception:
        return d
start, end = rows[0], rows[-1]
start_z = fl(start, 'mocap_z')
max_z = max(rows, key=lambda r: fl(r, 'mocap_z'))
max_drift = max(rows, key=lambda r: fl(r, 'horizontal_drift_m'))
max_err = max(rows, key=lambda r: fl(r, 'target_error_m', -1))
active = [r for r in rows if r.get('xy_control_active') in ('1', 'True', 'true')]
print('rows', len(rows), 'duration', fl(end, 'elapsed_s'))
print('z start/end/max/above', start_z, fl(end, 'mocap_z'), fl(max_z, 'mocap_z'), fl(max_z, 'mocap_z') - start_z)
print('thrust end/max', fl(end, 'thrust_raw'), max(fl(r, 'thrust_raw') for r in rows))
print('drift end/max', fl(end, 'horizontal_drift_m'), fl(max_drift, 'horizontal_drift_m'))
print('target err end/max', fl(end, 'target_error_m'), fl(max_err, 'target_error_m'))
print('active rows', len(active), 'first active', fl(active[0], 'elapsed_s') if active else None)
print('end row', {k: end.get(k) for k in ['elapsed_s','thrust_raw','mocap_x','mocap_y','mocap_z','horizontal_drift_m','target_error_m','velocity_x_m_s','velocity_y_m_s','roll_cmd_deg','pitch_cmd_deg','yaw_deg','battery_v','xy_control_active']})
PY
```

## Success Criteria Before Figure-8

Do not spend more time on figure-8 until hold-XY is repeatable.

Minimum next milestone:

- `hold-xy` mode
- height below `0.35m` above start
- drift stays under roughly `0.25m` for `10..15s`
- no stale mocap failures
- battery stays high enough that thrust authority remains predictable

After that:

- run a tiny, slow figure-8
- radius around `0.05..0.08m`
- period around `24s`
- trigger only after a stable hold

## Recommended Next Code Change

If the next hold run is still inconsistent, add manual activation keys:

- `h`: enable XY hold now and reset target to current mocap position
- `f`: start figure-8 now, only after XY hold is active

Reason:

- height-based activation works, but the user can visually judge when the drone
  is truly airborne and stable better than a noisy low-altitude threshold.
- manual figure-8 start avoids entering path tracking while the user is still
  fighting vertical thrust.

Keep automatic height activation as a fallback.

## Files To Know

- `session.md`: chronological project notes.
- `HANDOFF.md`: this handoff.
- `mocap_vertical_thrust_mapper.py`: active control/logging script.
- `keyboard_thrust_test.py`: simple keyboard raw-thrust test.
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guided-manual-flight-logger.py`: passive mocap/controller logger.
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guarded-takeoff.py`: conservative high-level takeoff test, not currently recommended.
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guarded-thrust-test.py`: older guarded raw-thrust test.

## Current Git State Caveat

The working tree has uncommitted and untracked files. Do not revert unrelated
changes. As of this handoff, expected changed/untracked paths include:

- `session.md`
- `HANDOFF.md`
- `mocap_vertical_thrust_mapper.py`
- `keyboard_thrust_test.py`
- `flight_logs/`
- `src/aimslab/crazyflie-clients-python/src/aimslab/examples/mocap-guided-manual-flight-logger.py`
- `src/cfclient/ui/tabs/flightreading.txt`
