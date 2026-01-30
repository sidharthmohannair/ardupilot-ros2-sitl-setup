#!/usr/bin/env python3

"""
Square Pattern Flight Example
==============================
This script demonstrates precise geometric navigation with position feedback:
1. Takeoff to 10 meters
2. Fly a 10m x 10m square pattern
3. Pause 2 seconds at each corner
4. Return to center
5. Land

Pattern (top view):
    2-------3
    |       |
    |   O   |  O = takeoff/landing
    |       |  Each side = 10m
    1-------4

Corners (ENU coordinates):
1. (10, 0, 10)
2. (10, 10, 10)
3. (0, 10, 10)
4. (0, 0, 10)

Author: Sidharth Mohan Nair https://github.com/sidharthmohannair | SimToFly https://github.com/simtofly
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time
import math


class SquarePatternController(Node):
    """
    Square pattern flight with position feedback control.
    """
    
    def __init__(self):
        super().__init__('square_pattern_controller')
        
        # State variables
        self.connected = False
        self.armed = False
        self.current_mode = ""
        
        # Current position (ENU frame)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # QoS profile for MAVROS topics (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        # Subscribe to local position
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.position_callback,
            qos_profile
        )
        
        # Setpoint publisher
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        self.get_logger().info('Square Pattern Controller initialized')
        self.wait_for_services()
    
    def state_callback(self, msg):
        """Update state"""
        self.connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode
    
    def position_callback(self, msg):
        """Update current position (ENU frame)"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
    
    def wait_for_services(self):
        """Wait for services"""
        self.get_logger().info('Waiting for services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.get_logger().info('Services ready')
    
    def wait_for_connection(self, timeout=30):
        """Wait for connection"""
        self.get_logger().info('Waiting for connection...')
        start_time = time.time()
        while not self.connected:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Connected')
        return True
    
    def set_mode(self, mode, timeout=10):
        """Change flight mode"""
        self.get_logger().info(f'Setting mode: {mode}')
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().mode_sent:
            return False
        
        start_time = time.time()
        while self.current_mode != mode:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info(f'Mode: {mode}')
        return True
    
    def arm(self, timeout=10):
        """Arm motors"""
        self.get_logger().info('Arming...')
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().success:
            return False
        
        start_time = time.time()
        while not self.armed:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Armed')
        return True
    
    def takeoff(self, altitude):
        """Takeoff to altitude"""
        self.get_logger().info(f'Takeoff to {altitude}m')
        request = CommandTOL.Request()
        request.altitude = altitude
        
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().success:
            return False
        
        self.get_logger().info('Takeoff command sent')
        return True
    
    def send_position(self, x, y, z):
        """
        Send position setpoint.
        
        Args:
            x: East position (meters)
            y: North position (meters)
            z: Up position (meters)
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Ensure all values are floats
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(msg)
    
    def get_distance_to_target(self, target_x, target_y, target_z):
        """Calculate 3D distance to target"""
        dx = self.current_x - target_x
        dy = self.current_y - target_y
        dz = self.current_z - target_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def fly_to_position(self, x, y, z, corner_name, tolerance=0.5, timeout=30):
        """
        Fly to position and wait until reached.
        
        Args:
            x, y, z: Target position (ENU frame)
            corner_name: Name for logging (e.g., "Corner 1")
            tolerance: Acceptable error in meters
            timeout: Maximum time to reach position
            
        Returns:
            bool: True if reached, False if timeout
        """
        self.get_logger().info(f'Flying to {corner_name}: ({x}, {y}, {z})')
        
        # Send position command
        self.send_position(x, y, z)
        
        # Wait until position reached
        start_time = time.time()
        while True:
            distance = self.get_distance_to_target(x, y, z)
            
            # Check if reached (within tolerance)
            if distance < tolerance:
                self.get_logger().info(
                    f'{corner_name} reached! '
                    f'Position: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})'
                )
                return True
            
            # Check timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f'{corner_name} timeout! Distance: {distance:.2f}m'
                )
                return False
            
            # Continue checking
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def pause_at_corner(self, duration):
        """Pause at corner for specified duration"""
        self.get_logger().info(f'Pausing for {duration} seconds...')
        time.sleep(duration)
    
    def land(self):
        """Land at current position"""
        self.get_logger().info('Landing...')
        return self.set_mode('LAND')
    
    def execute_mission(self):
        """
        Execute square pattern flight mission.
        
        Square pattern (10m sides):
        1. (10, 0, 10) - East
        2. (10, 10, 10) - Northeast
        3. (0, 10, 10) - North
        4. (0, 0, 10) - Center
        """
        self.get_logger().info('=== Starting Square Pattern Mission ===')
        
        # Define waypoints (corners of square)
        waypoints = [
            (10, 0, 10, "Corner 1 (East)"),
            (10, 10, 10, "Corner 2 (Northeast)"),
            (0, 10, 10, "Corner 3 (North)"),
            (0, 0, 10, "Corner 4 (Center)")
        ]
        
        # Step 1: Connect
        if not self.wait_for_connection():
            return False
        
        # Step 2: Set GUIDED mode
        if not self.set_mode('GUIDED'):
            return False
        
        # Step 3: Arm
        if not self.arm():
            return False
        
        # Step 4: Takeoff to 10m
        if not self.takeoff(altitude=10.0):
            return False
        
        # Wait for altitude stabilization
        self.get_logger().info('Stabilizing altitude...')
        time.sleep(10)
        
        # Step 5: Fly square pattern
        self.get_logger().info('Starting square pattern...')
        
        for x, y, z, corner_name in waypoints:
            # Fly to corner
            if not self.fly_to_position(x, y, z, corner_name):
                return False
            
            # Pause at corner
            self.pause_at_corner(duration=2)
        
        self.get_logger().info('Square pattern complete!')
        
        # Step 6: Land
        if not self.land():
            return False
        
        self.get_logger().info('=== Mission Complete ===')
        return True


def main():
    """Main entry point"""
    rclpy.init()
    controller = SquarePatternController()
    
    try:
        success = controller.execute_mission()
        
        if success:
            controller.get_logger().info('Mission succeeded')
        else:
            controller.get_logger().error('Mission failed')
        
        # Wait for landing
        time.sleep(15)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted')
    except Exception as e:
        controller.get_logger().error(f'Error: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
