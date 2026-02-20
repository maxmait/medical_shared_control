#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, WrenchStamped
import evdev
from evdev import ecodes, ff, InputDevice
import threading
import time

class HapticFeedbackNode(Node):
    def __init__(self):
        super().__init__('haptic_feedback_node')
        
        # Parameters
        self.declare_parameter('controller_device', '/dev/input/event9')
        self.declare_parameter('max_vibration_intensity', 32000)  # 0-65535
        self.declare_parameter('enable_collision_feedback', True)
        self.declare_parameter('enable_force_feedback', True)
        self.declare_parameter('enable_status_feedback', True)
        
        device_path = self.get_parameter('controller_device').get_parameter_value().string_value
        self.max_intensity = self.get_parameter('max_vibration_intensity').get_parameter_value().integer_value
        
        # Initialize controller
        try:
            self.controller = InputDevice(device_path)
            self.get_logger().info(f"Haptic controller initialized: {self.controller.name}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize controller: {e}")
            self.controller = None
            return
        
        # Subscriptions for feedback triggers
        self.collision_sub = self.create_subscription(
            Bool, '/robot_collision_detected', self.collision_callback, 10)
        
        self.force_sub = self.create_subscription(
            WrenchStamped, '/robot_force_feedback', self.force_callback, 10)
        
        self.velocity_sub = self.create_subscription(
            Twist, '/panda_velocity_cmd', self.velocity_callback, 10)
        
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)
        
        # Status feedback
        self.mode_change_sub = self.create_subscription(
            Bool, '/teleop_mode_changed', self.mode_change_callback, 10)
        
        # Vibration control
        self.current_effect_id = None
        self.vibration_lock = threading.Lock()
        
        self.get_logger().info("Haptic feedback node started")
        
        # Test vibration on startup
        self.test_vibration()
    
    def play_vibration(self, strong_intensity, weak_intensity, duration_ms=200):
        """Play vibration effect with given intensities"""
        if not self.controller:
            return
            
        with self.vibration_lock:
            try:
                # Stop current effect if any
                if self.current_effect_id is not None:
                    self.controller.erase_effect(self.current_effect_id)
                
                # Clamp intensities
                strong_intensity = max(0, min(65535, int(strong_intensity)))
                weak_intensity = max(0, min(65535, int(weak_intensity)))
                
                # Create rumble effect
                effect_type = ff.EffectType(
                    ff_rumble_effect=ff.Rumble(
                        strong_magnitude=strong_intensity, 
                        weak_magnitude=weak_intensity
                    )
                )
                
                effect = ff.Effect(
                    ecodes.FF_RUMBLE,
                    -1,  # new effect
                    0,   # direction
                    ff.Trigger(0, 0),
                    ff.Replay(duration_ms, 0),
                    effect_type
                )
                
                # Upload and play effect
                self.current_effect_id = self.controller.upload_effect(effect)
                self.controller.write(ecodes.EV_FF, self.current_effect_id, 1)
                
                # Clean up after duration
                def cleanup():
                    time.sleep(duration_ms / 1000 + 0.1)
                    if self.current_effect_id is not None:
                        try:
                            self.controller.erase_effect(self.current_effect_id)
                            self.current_effect_id = None
                        except:
                            pass
                
                threading.Thread(target=cleanup, daemon=True).start()
                
            except Exception as e:
                self.get_logger().warn(f"Vibration failed: {e}")
    
    def collision_callback(self, msg):
        """Handle collision detection feedback"""
        if msg.data and self.get_parameter('enable_collision_feedback').get_parameter_value().bool_value:
            self.get_logger().warn("COLLISION DETECTED - Strong vibration!")
            # Strong collision feedback - both motors at high intensity
            self.play_vibration(self.max_intensity, self.max_intensity, duration_ms=1000)
    
    def force_callback(self, msg):
        """Handle force feedback from robot sensors"""
        if not self.get_parameter('enable_force_feedback').get_parameter_value().bool_value:
            return
            
        # Calculate force magnitude
        force_mag = (msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)**0.5
        
        # Map force to vibration intensity (adjust thresholds as needed)
        if force_mag > 10.0:  # High force threshold
            intensity = min(self.max_intensity, int(force_mag * 1000))
            self.play_vibration(intensity // 2, intensity, duration_ms=100)
        elif force_mag > 5.0:  # Medium force threshold  
            intensity = min(self.max_intensity // 2, int(force_mag * 500))
            self.play_vibration(0, intensity, duration_ms=100)
    
    def velocity_callback(self, msg):
        """Provide subtle feedback based on velocity commands"""
        # Very light feedback to indicate motion is active
        velocity_mag = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        
        if velocity_mag > 0.1:  # Only if significant motion
            # Very light vibration to indicate active control
            weak_intensity = min(8000, int(velocity_mag * 2000))
            self.play_vibration(0, weak_intensity, duration_ms=50)
    
    def emergency_callback(self, msg):
        """Handle emergency stop feedback"""
        if msg.data:
            self.get_logger().error("EMERGENCY STOP - Maximum vibration!")
            # Maximum intensity emergency feedback
            for _ in range(3):  # Pulse 3 times
                self.play_vibration(65535, 65535, duration_ms=300)
                time.sleep(0.1)
    
    def mode_change_callback(self, msg):
        """Feedback when switching coordinate frames or modes"""
        if self.get_parameter('enable_status_feedback').get_parameter_value().bool_value:
            # Short pulse to indicate mode change
            self.play_vibration(16000, 16000, duration_ms=250)
            self.get_logger().info("Mode changed - feedback pulse")

    def test_vibration(self):
        """Test vibration on startup"""
        self.get_logger().info("Testing haptic feedback...")
        
        # Quick startup test
        self.play_vibration(20000, 0, duration_ms=200)      # Strong motor
        time.sleep(0.3)
        self.play_vibration(0, 20000, duration_ms=200)      # Weak motor
        time.sleep(0.3)
        self.play_vibration(15000, 15000, duration_ms=200)  # Both motors
        
        self.get_logger().info("Haptic feedback test complete!")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        haptic_node = HapticFeedbackNode()
        rclpy.spin(haptic_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()