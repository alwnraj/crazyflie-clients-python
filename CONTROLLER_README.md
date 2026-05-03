# Logitech F310 Controller for Crazyflie

## ✅ Status: WORKING!

Your **Logitech F310 Gamepad** is detected and ready to use!

```
Device: /dev/input/js1
Model:  Logitech F310 Gamepad [DirectInput Mode]
Status: ✓ OPERATIONAL
Axes: 6 | Buttons: 12
```

**Important**: Make sure the switch on the **back of your F310** is set to **'D'** (DirectInput mode), not 'X'.

---

## 🚀 Quick Start (3 Steps)

### Step 1: Verify Controller
```bash
./RUN_THIS_FIRST.sh
```
Quick check that controller is connected and readable.

### Step 2: Test Controller Input ⭐
```bash
python3 demo_controller_live.py
```
**DO THIS FIRST!** Shows live dashboard with Roll, Pitch, Yaw, Thrust values. Move sticks and press buttons to verify everything works.

### Step 3: Fly Your Crazyflie! 🚁
```bash
python3 test_flight_with_controller.py
```
Full manual flight control via gamepad. Requires Crazyflie connected.

---

## 📋 Available Scripts

| Script | Purpose | When to Use |
|--------|---------|-------------|
| **demo_controller_live.py** ⭐ | Live visual dashboard | **Start here** - Test controller before flying |
| **test_flight_with_controller.py** 🚁 | Fly with controller | Manual flight control of Crazyflie |
| check_controller.py | Quick detection | Fast check if controller works |
| test_logitech_controller.py | Raw events | Debug controller issues |
| RUN_THIS_FIRST.sh | Verification | Quick system check |

---

## 🎮 Controller Layout & Controls

```
        [LB]                          [RB]
         |                             |
    _____|_____                   _____|_____
   /  ↑      \                   /     ↑    \
  |  ←•→      |                 |     ←•→    |
  |   ↓       |                 |      ↓     |
   \_________/                   \_________/
  LEFT STICK                    RIGHT STICK
  Roll & Pitch                  Yaw & Thrust

     [Back]  [Start]
        
       [Y]
    [X]   [B]      
       [A]
```

### Flight Control Mapping

| Control | Input | Range | Effect |
|---------|-------|-------|--------|
| **Roll** | Left Stick ← → | -30° to +30° | Tilt left/right |
| **Pitch** | Left Stick ↑ ↓ | -30° to +30° | Tilt forward/back |
| **Yaw** | Right Stick ← → | -200°/s to +200°/s | Rotate left/right |
| **Thrust** | Right Stick ↑ ↓ | 0 to 60000 | Go up/down |
| **Emergency Stop** | START Button | - | Cut motors immediately |
| **Alt Hold** | RB Button | - | Maintain altitude (future) |

**In Simple Terms:**
- **Left Stick** = Direction (tilt the drone)
- **Right Stick Up/Down** = Go Up / Go Down
- **Right Stick Left/Right** = Spin Left / Spin Right

---

## ✈️ Flying Instructions

### Before Flight Checklist
- [ ] Controller tested with `demo_controller_live.py`
- [ ] Battery charged to 4.0V+
- [ ] Clear flight area (2m radius minimum)
- [ ] Know emergency stop button (START)
- [ ] Thrust starts at zero (right stick down)
- [ ] F310 switch set to 'D' (DirectInput mode)

### Takeoff Procedure
1. **Start with thrust at minimum** (right stick all the way down)
2. Run `python3 test_flight_with_controller.py`
3. Wait for connection and arming
4. **Slowly** push the right stick up to increase thrust
5. Around 40-50% thrust, the drone should lift off
6. Use left stick to control direction
7. Use right stick left/right to rotate
8. **Press START button for emergency stop**

### Safety Tips
- ⚠️ **Always start with thrust at zero** - Right stick down before arming
- ⚠️ **Increase thrust slowly** - Don't slam the stick to full power
- ⚠️ **Small movements** - Tiny stick movements have big effects
- ⚠️ **Know your emergency stop** - START button cuts motors immediately
- ⚠️ **Practice in open space** - At least 2 meters of clear space
- ⚠️ **Monitor battery** - Script shows voltage, charge to 4.0V+ for best results

---

## 🔧 Troubleshooting

### Controller Not Detected
```bash
# Check USB connection
lsusb | grep -i logitech

# Check joystick devices
ls -la /dev/input/js*

# Run verification
./RUN_THIS_FIRST.sh
```

### Permission Denied
```bash
# Check if you're in input group
groups | grep input

# If not, add yourself
sudo usermod -a -G input $USER
# Then log out and log back in
```

### Wrong Device Path
The scripts prefer `/dev/input/js1`, then fall back to the first available
`/dev/input/js*` device. If you need to force a specific device, edit
`CONTROLLER_DEVICE` in the scripts.

### Controller Not Responding
- Check switch on back of F310 is set to **'D'** (DirectInput)
- Try unplugging and replugging the controller
- Run `python3 demo_controller_live.py` and move sticks

### Using with cfclient GUI
The Crazyflie client has built-in support:
```bash
cfclient
```
Then:
1. Go to Input Device menu
2. Select your controller (should appear as "Logitech Gamepad F310")
3. Choose the "xbox360_mode1" configuration
4. Start flying!

---

## 🆘 Emergency Procedures

| Situation | Action |
|-----------|--------|
| Drone out of control | **Press START button** immediately |
| Stuck in corner/wall | **Press START button** |
| Low battery warning | Land immediately (thrust down) |
| Lost orientation | **Press START button**, restart |
| Any doubt | **Press START button** |

The START button sends emergency stop command - motors cut immediately.

---

## 📊 Technical Details

### Button/Axis Mapping

**Axes:**
- Axis 0: Left Stick X (left/right)
- Axis 1: Left Stick Y (up/down)
- Axis 2: Right Stick X (left/right)
- Axis 3: Right Stick Y (up/down)
- Axis 4: L2/R2 Triggers
- Axis 5: D-Pad

**Buttons:**
- Button 0: X
- Button 1: A
- Button 2: B
- Button 3: Y
- Button 4: LB (Left Bumper)
- Button 5: RB (Right Bumper) - Alt Hold
- Button 6: LT (Left Trigger)
- Button 7: RT (Right Trigger)
- Button 8: Back
- Button 9: Start - Emergency Stop
- Button 10: Left Stick Click
- Button 11: Right Stick Click

### Flight Control Values

- **Roll**: -30° to +30° (negative = left, positive = right)
- **Pitch**: -30° to +30° (negative = backward, positive = forward)
- **Yaw**: -200°/s to +200°/s (negative = CCW, positive = CW)
- **Thrust**: 0 to 60000 (0 = off, ~35000 = hover, 60000 = max)

### Deadzone
A 10% deadzone is active to prevent drift from small stick movements.

---

## 💡 Pro Tips

1. **Practice without the drone first** - Use `demo_controller_live.py` to get familiar
2. **Smooth movements** - Don't jerk the sticks
3. **Center sticks to hover** - Release sticks to their center position
4. **Thrust is vertical only** - Right stick up/down controls altitude
5. **Small corrections** - Tiny stick movements have big effects

---

## 📁 Files Created

**Scripts:**
- `demo_controller_live.py` - Live visual dashboard (recommended first)
- `test_flight_with_controller.py` - Fly with controller
- `check_controller.py` - Quick detection test
- `test_logitech_controller.py` - Raw event monitor
- `RUN_THIS_FIRST.sh` - Quick verification

**Documentation:**
- `CONTROLLER_README.md` - This file (complete guide)
- `START_HERE.txt` - Quick reference card

---

## 🎯 Recommended Workflow

```
1. First Time Setup:
   ./RUN_THIS_FIRST.sh
   └─> Verifies controller is ready

2. Learn the Controls:
   python3 demo_controller_live.py
   └─> Move sticks, see what happens
   └─> Get comfortable with the layout

3. Connect Crazyflie:
   python3 test_flight_with_controller.py
   └─> Follow on-screen instructions
   └─> Start with thrust at zero
   └─> Slowly increase thrust to lift off

4. Practice Flying:
   └─> Small movements
   └─> Hover in place
   └─> Gentle turns
   └─> Controlled landing
```

---

## ✨ You're All Set!

Your controller is **configured, tested, and ready to fly**!

**Start here:**
```bash
python3 demo_controller_live.py
```

**Then fly:**
```bash
python3 test_flight_with_controller.py
```

**Happy Flying! 🚁**

---

*Controller: Logitech F310 Gamepad (046d:c216)*  
*Linux Native Support - No additional drivers required*
