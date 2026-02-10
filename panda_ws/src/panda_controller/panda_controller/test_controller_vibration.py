#!/usr/bin/env python3
# filepath: /home/max/medical_robot_shared_control/panda_ws/src/panda_controller/panda_controller/test_controller_vibration.py

import evdev
from evdev import ecodes, ff, InputDevice
import time
import sys
import os

def main():
    print("Testing Controller Vibration")
    print("=" * 40)
    
    # Try to find the controller device
    controller_device = None
    possible_devices = ['/dev/input/event9', '/dev/input/js0', '/dev/input/event8', '/dev/input/event10']
    
    for device_path in possible_devices:
        try:
            if os.path.exists(device_path):
                dev = InputDevice(device_path)
                # Check if device supports force feedback
                if ecodes.EV_FF in dev.capabilities():
                    controller_device = device_path
                    print(f"Found haptic controller: {dev.name} at {device_path}")
                    break
                else:
                    print(f"Device {device_path} doesn't support force feedback")
        except Exception as e:
            print(f"Cannot access {device_path}: {e}")
    
    if not controller_device:
        print("No haptic-capable controller found!")
        print("Try these commands to find your controller:")
        print("   ls /dev/input/")
        print("   evtest  # (install with: sudo apt install evtest)")
        return
    
    try:
        dev = InputDevice(controller_device)
    except Exception as e:
        print(f"Failed to initialize controller: {e}")
        print("Try running with sudo or check permissions")
        return

    # Function to play a rumble effect
    def play_rumble(strong, weak, duration_ms=500):
        try:
            effect_type = ff.EffectType(
                ff_rumble_effect=ff.Rumble(strong_magnitude=strong, weak_magnitude=weak)
            )

            effect = ff.Effect(
                ecodes.FF_RUMBLE,    # type
                -1,                  # new effect
                0,                   # direction
                ff.Trigger(0, 0),    # trigger
                ff.Replay(duration_ms, 0),  # duration
                effect_type
            )

            effect_id = dev.upload_effect(effect)
            dev.write(ecodes.EV_FF, effect_id, 1)
            time.sleep(duration_ms / 1000 + 0.1)  # wait for effect to finish
            dev.erase_effect(effect_id)
            return True
        except Exception as e:
            print(f"Vibration failed: {e}")
            return False

    print("\nStarting vibration test...")
    
    # Quick test first
    print("Quick test - both motors medium intensity...")
    if not play_rumble(20000, 20000, duration_ms=500):
        print("Basic vibration test failed!")
        return
    
    print("Basic vibration works!")
    time.sleep(1)
    
    # Gradually increase strong motor
    print("\nTesting strong motor (left)...")
    for i, strong in enumerate(range(0, 65536, 8192)):
        weak = 0  # only strong motor
        print(f"  Step {i+1}/8 - Strong: {strong:5d}, Weak: {weak:5d}")
        play_rumble(strong, weak, duration_ms=800)
        time.sleep(0.2)

    time.sleep(1)
    
    # Gradually increase weak motor  
    print("\nTesting weak motor (right)...")
    for i, weak in enumerate(range(0, 65536, 8192)):
        strong = 0  # only weak motor
        print(f"  Step {i+1}/8 - Strong: {strong:5d}, Weak: {weak:5d}")
        play_rumble(strong, weak, duration_ms=800)
        time.sleep(0.2)

    time.sleep(1)
    
    # Gradually increase both motors together
    print("\nTesting both motors together...")
    for i, val in enumerate(range(0, 65536, 8192)):
        strong = val
        weak = val
        print(f"  Step {i+1}/8 - Strong: {strong:5d}, Weak: {weak:5d}")
        play_rumble(strong, weak, duration_ms=800)
        time.sleep(0.2)

    time.sleep(1)
    
    # Test different patterns
    print("\nTesting vibration patterns...")
    
    # Emergency pattern
    print("Emergency pattern (3 strong pulses)...")
    for _ in range(3):
        play_rumble(65535, 65535, duration_ms=300)
        time.sleep(0.1)
    
    time.sleep(1)
    
    # Mode change pattern
    print("Mode change pattern (short pulse)...")
    play_rumble(30000, 30000, duration_ms=150)
    
    time.sleep(1)
    
    # Force feedback simulation
    print("Force feedback simulation (variable intensity)...")
    for intensity in [10000, 20000, 40000, 20000, 10000]:
        play_rumble(intensity // 2, intensity, duration_ms=200)
        time.sleep(0.1)

    print("\nVibration test complete!")
    print("If you felt all the vibrations, haptic feedback is working correctly!")

if __name__ == '__main__':
    main()