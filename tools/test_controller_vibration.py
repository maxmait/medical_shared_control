import evdev
from evdev import ecodes, ff, InputDevice
import time

# USB-connected controller
dev = InputDevice('/dev/input/event9')

# Function to play a rumble effect
def play_rumble(strong, weak, duration_ms=500):
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

# Gradually increase strong motor
print("Testing strong motor (left)...")
for strong in range(0, 65536, 8192):  # 0 → 65535 in steps
    weak = 0  # only strong motor
    print(f"Strong: {strong}, Weak: {weak}")
    play_rumble(strong, weak, duration_ms=1000)

# Gradually increase weak motor
print("Testing weak motor (right)...")
for weak in range(0, 65536, 8192):
    strong = 0  # only weak motor
    print(f"Strong: {strong}, Weak: {weak}")
    play_rumble(strong, weak, duration_ms=1000)

# Gradually increase both motors together
print("Testing both motors...")
for val in range(0, 65536, 8192):
    strong = val
    weak = val
    print(f"Strong: {strong}, Weak: {weak}")
    play_rumble(strong, weak, duration_ms=1000)

print("Test complete!")
