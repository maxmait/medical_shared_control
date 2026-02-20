#!/usr/bin/env python3
# filepath: /home/max/medical_robot_shared_control/panda_ws/src/panda_controller/panda_controller/test_controller_vibration.py

import evdev
from evdev import ecodes, ff, InputDevice
import time
import sys
import os

def find_haptic_controller():
    """Find controller that supports haptic feedback by checking /proc/bus/input/devices"""
    try:
        with open('/proc/bus/input/devices', 'r') as f:
            content = f.read()
        
        # Split into device blocks
        devices = content.split('\n\n')
        
        for device_block in devices:
            if not device_block.strip():
                continue
                
            lines = device_block.strip().split('\n')
            device_info = {}
            
            for line in lines:
                if line.startswith('N: '):
                    device_info['name'] = line.split('Name="')[1].rstrip('"') if 'Name="' in line else ''
                elif line.startswith('H: '):
                    handlers = line.split('Handlers=')[1] if 'Handlers=' in line else ''
                    device_info['handlers'] = handlers
                elif line.startswith('B: FF='):
                    ff_capabilities = line.split('B: FF=')[1] if 'B: FF=' in line else ''
                    device_info['ff'] = ff_capabilities
            
            # Check if this device has force feedback and event handler
            if ('ff' in device_info and 
                device_info['ff'] != '0' and 
                'handlers' in device_info and 
                'event' in device_info['handlers']):
                
                # Extract event number
                handlers = device_info['handlers'].split()
                for handler in handlers:
                    if handler.startswith('event'):
                        device_path = f'/dev/input/{handler}'
                        if os.path.exists(device_path):
                            print(f"Auto-detected haptic controller: {device_info.get('name', 'Unknown')} at {device_path}")
                            return device_path
    except Exception as e:
        print(f"Auto-detection failed: {e}")
    
    return None

def scan_all_devices():
    """Scan all input devices and categorize them"""
    print("Scanning all input devices...")
    print("=" * 50)
    
    haptic_devices = []
    controller_devices = []
    other_devices = []
    
    # Check a wide range of event devices
    for i in range(32):
        device_path = f'/dev/input/event{i}'
        
        if not os.path.exists(device_path):
            continue
            
        try:
            device = InputDevice(device_path)
            capabilities = device.capabilities()
            
            # Check device capabilities
            has_ff = ecodes.EV_FF in capabilities
            has_buttons = ecodes.EV_KEY in capabilities
            has_axes = ecodes.EV_ABS in capabilities
            has_rel = ecodes.EV_REL in capabilities  # Mouse/trackpad
            
            device_info = {
                'path': device_path,
                'name': device.name,
                'has_haptic': has_ff,
                'has_buttons': has_buttons,
                'has_axes': has_axes,
                'has_relative': has_rel
            }
            
            # Categorize the device
            if has_ff and has_buttons and has_axes:
                haptic_devices.append(device_info)
                print(f"HAPTIC GAMEPAD: {device_path}")
                print(f"     Name: {device.name}")
                if has_ff:
                    ff_effects = capabilities.get(ecodes.EV_FF, [])
                    effect_names = [ecodes.FF[effect] for effect in ff_effects if effect in ecodes.FF]
                    print(f"     Effects: {effect_names}")
                print()
                
            elif has_buttons and has_axes and not has_ff:
                controller_devices.append(device_info)
                print(f"GAMEPAD (no haptic): {device_path}")
                print(f"     Name: {device.name}")
                print()
                
            elif has_buttons or has_axes or has_rel:
                other_devices.append(device_info)
                device_type = []
                if has_buttons: device_type.append("buttons")
                if has_axes: device_type.append("axes") 
                if has_rel: device_type.append("relative")
                
                print(f"OTHER INPUT: {device_path}")
                print(f"     Name: {device.name}")
                print(f"     Type: {', '.join(device_type)}")
                print()
        
        except PermissionError:
            print(f"PERMISSION DENIED: {device_path}")
        except Exception as e:
            # Skip devices that can't be opened (not input devices)
            continue
    
    return haptic_devices, controller_devices, other_devices

def test_haptic_device(device_path, device_name):
    """Test haptic feedback on a specific device"""
    print(f"\nTesting haptic feedback on: {device_name}")
    print(f"   Device: {device_path}")
    print("-" * 40)
    
    try:
        dev = InputDevice(device_path)
    except Exception as e:
        print(f"Failed to open device: {e}")
        return False
    
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

    # Quick test first
    print("Quick test - medium intensity...")
    if not play_rumble(25000, 25000, duration_ms=500):
        print("Basic vibration test failed!")
        return False
    
    print("Basic vibration works!")
    time.sleep(0.5)
    
    # Test different intensities
    print("Testing different intensities...")
    intensities = [10000, 25000, 40000, 25000, 10000]
    for i, intensity in enumerate(intensities):
        print(f"   Intensity {i+1}/5: {intensity}")
        play_rumble(intensity, intensity, duration_ms=300)
        time.sleep(0.2)
    
    # Test emergency pattern
    print("Testing emergency pattern...")
    for i in range(3):
        play_rumble(65535, 65535, duration_ms=200)
        time.sleep(0.1)
    
    print("All haptic tests passed!")
    return True

def main():
    print("Controller Vibration Test Suite")
    print("=" * 50)
    
    # First, try auto-detection
    auto_detected = find_haptic_controller()
    if auto_detected:
        print("Auto-detection successful!")
        if test_haptic_device(auto_detected, "Auto-detected controller"):
            print(f"\nSUCCESS! Your haptic controller is working perfectly!")
            print(f"Use this device path in your launch file: {auto_detected}")
            return
    
    # If auto-detection failed, scan all devices
    print("\nAuto-detection failed, scanning all devices...")
    haptic_devices, controller_devices, other_devices = scan_all_devices()
    
    if not haptic_devices and not controller_devices:
        print("No gamepads or controllers found!")
        print("Make sure your controller is:")
        print("   - Plugged in (USB) or connected (Bluetooth)")
        print("   - Recognized by the system")
        print("   - Try: ls /dev/input/ | grep event")
        return
    
    if haptic_devices:
        print(f"Found {len(haptic_devices)} haptic controller(s)!")
        for device in haptic_devices:
            success = test_haptic_device(device['path'], device['name'])
            if success:
                print(f"\nSUCCESS! Haptic controller working!")
                print(f"Use this device path: {device['path']}")
                return
    
    if controller_devices:
        print(f"\nFound {len(controller_devices)} controller(s) without haptic feedback:")
        for device in controller_devices:
            print(f"   {device['path']}: {device['name']}")
        print("These controllers work for input but don't support vibration")

    if other_devices:
        print(f"\nFound {len(other_devices)} other input devices:")
        for device in other_devices:
            print(f"   {device['path']}: {device['name']}")
    
    print("\nNo working haptic controllers found!")
    print("Troubleshooting:")
    print("   1. Check controller connection")
    print("   2. Try: sudo chmod 666 /dev/input/event*")
    print("   3. Add user to input group: sudo usermod -a -G input $USER")
    print("   4. Check with: evtest")

if __name__ == '__main__':
    main()