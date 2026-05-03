#!/bin/bash
#
# Quick controller verification script
# Run this first to test your Logitech F310 controller
#

echo "============================================================"
echo "  Logitech F310 Controller - Quick Verification"
echo "============================================================"
echo ""

# Check if controller device exists. Prefer js1 for the lab F310 setup, but
# accept the first joystick device if Linux assigns a different number.
CONTROLLER_DEVICE="/dev/input/js1"
if [ ! -e "$CONTROLLER_DEVICE" ]; then
    CONTROLLER_DEVICE="$(ls /dev/input/js* 2>/dev/null | head -n 1)"
fi

if [ -n "$CONTROLLER_DEVICE" ] && [ -e "$CONTROLLER_DEVICE" ]; then
    echo "✓ Controller found at $CONTROLLER_DEVICE"
else
    echo "✗ Controller not found"
    echo ""
    echo "Available joystick devices:"
    ls -la /dev/input/js* 2>/dev/null || echo "  None found"
    echo ""
    exit 1
fi

# Check permissions
if [ -r "$CONTROLLER_DEVICE" ]; then
    echo "✓ Controller is readable (permissions OK)"
else
    echo "✗ Cannot read controller (permission issue)"
    echo "  Try: sudo usermod -a -G input $USER"
    echo "  Then log out and log back in"
    exit 1
fi

# Check USB
echo ""
echo "USB Device Info:"
lsusb | grep -i logitech || echo "  Logitech device not found in USB list"

echo ""
echo "============================================================"
echo "  Controller is ready!"
echo "============================================================"
echo ""
echo "Next steps:"
echo ""
echo "  1. Test controller with live visual feedback:"
echo "     python3 demo_controller_live.py"
echo ""
echo "  2. Fly your Crazyflie with the controller:"
echo "     python3 test_flight_with_controller.py"
echo ""
echo "  3. Read the quick start guide:"
echo "     cat CONTROLLER_QUICK_START.md"
echo ""
echo "============================================================"
