#!/usr/bin/env python3
"""
Quick test script to verify X11/X server connectivity
Run this to check if your SSH session has proper X11 forwarding enabled
"""

import os
import sys

def test_x11():
    """Test X11 connectivity"""
    print("=" * 60)
    print("X11 Forwarding Test")
    print("=" * 60)
    
    # Check DISPLAY variable
    display = os.environ.get('DISPLAY', '')
    print(f"\n1. DISPLAY variable: {display if display else '(NOT SET)'}")
    
    if not display:
        print("   ❌ DISPLAY is not set!")
        print("   → X11 forwarding is NOT working")
        return False
    
    print(f"   ✓ DISPLAY is set to: {display}")
    
    # Check SSH connection
    ssh_client = os.environ.get('SSH_CLIENT', '')
    if ssh_client:
        print(f"\n2. SSH connection detected: {ssh_client}")
        print("   ✓ Connected via SSH")
    else:
        print("\n2. Not connected via SSH (local session)")
    
    # Try to import and test pynput
    print("\n3. Testing pynput import...")
    try:
        from pynput import keyboard
        print("   ✓ pynput imported successfully")
        
        # Try to create a listener (this will fail if X11 is not working)
        print("\n4. Testing X11 connection...")
        try:
            # Just try to access the backend - this will fail if X11 is broken
            listener = keyboard.Listener(on_press=lambda k: None)
            listener.start()
            listener.stop()
            print("   ✓ X11 connection working!")
            print("   ✓ Keyboard input should work")
            return True
        except Exception as e:
            error_msg = str(e)
            if 'X connection' in error_msg or 'DISPLAY' in error_msg:
                print(f"   ❌ X11 connection failed: {error_msg}")
                print("   → X11 forwarding is NOT working properly")
                return False
            else:
                print(f"   ⚠ Unexpected error: {e}")
                return False
                
    except ImportError as e:
        print(f"   ❌ pynput not installed: {e}")
        print("   → Install with: pip install pynput")
        return False
    except Exception as e:
        print(f"   ❌ Error: {e}")
        return False

if __name__ == '__main__':
    success = test_x11()
    print("\n" + "=" * 60)
    if success:
        print("✓ All tests passed! X11 forwarding is working.")
        print("  You should be able to run keyboard_flight_control.py")
    else:
        print("✗ X11 forwarding is NOT working properly")
        print("\nTo fix:")
        print("  1. Reconnect with X11 forwarding:")
        print("     ssh -X username@hostname")
        print("     (or ssh -Y for trusted X11 forwarding)")
        print("  2. Check SSH server config allows X11 forwarding")
        print("  3. If on local machine, set DISPLAY:")
        print("     export DISPLAY=:0")
    print("=" * 60)
    sys.exit(0 if success else 1)
