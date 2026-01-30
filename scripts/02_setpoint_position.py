#!/usr/bin/env python3

"""
Setpoint Position Control Example
==================================
This script demonstrates position control using setpoint commands:
1. Takeoff to 10 meters
2. Navigate to position (10, 20, 10) - 10m East, 20m North
3. Hold position for 10 seconds
4. Return to start position (0, 0, 10)
5. Land

Position coordinates are in ENU (East-North-Up) frame relative to takeoff point.

Author: SimToFly
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time
import math


class SetpointController(Node):
    """
    Position control using MAVROS setpoint commands.
    """
    
    def __init__(self):
        super().__init__('setpoint_controller')
        
        # State variables
        self.current_state = None
        self.connected = False
        self.armed = False
        self.current_mode = ""
        
        # Current position (ENU frame)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # QoS profile for MAVROS topics (they use BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to drone state
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
        
        # Publisher for position setpoints
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # Create service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        self.get_logger().info('Setpoint Controller initialized')
        self.wait_for_services()
    
    def state_callback(self, msg):
        """Update current drone state"""
        self.current_state = msg
        self.connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode
    
    def position_callback(self, msg):
        """
        Update current position from local_position topic.
        Position is in ENU frame (East-North-Up).
        """
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
    
    def wait_for_services(self):
        """Wait for all required services"""
        self.get_logger().info('Waiting for services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.get_logger().info('All services ready')
    
    def wait_for_connection(self, timeout=30):
        """Wait for MAVROS connection"""
        self.get_logger().info('Waiting for connection...')
        start_time = time.time()
        while not self.connected:
            if time.time() - start_time > timeout:
                self.get_logger().error('Connection timeout')
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
        """Takeoff to specified altitude"""
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
        Send position setpoint command.
        
        Args:
            x: Target X position (East) in meters
            y: Target Y position (North) in meters
            z: Target Z position (Up) in meters
        
        Note: Setpoints are latched - the drone will continue to target
              this position even if we stop publishing.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set target position (ENU frame)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Orientation (0, 0, 0, 1 = no rotation)
        msg.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(msg)
    
    def get_distance_to_target(self, target_x, target_y, target_z):
        """
        Calculate 3D distance to target position.
        
        Returns:
            float: Distance in meters
        """
        dx = self.current_x - target_x
        dy = self.current_y - target_y
        dz = self.current_z - target_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def wait_for_position(self, target_x, target_y, target_z, tolerance=0.5, timeout=30):
        """
        Wait until drone reaches target position.
        
        Args:
            target_x, target_y, target_z: Target position
            tolerance: Acceptable distance error in meters
            timeout: Maximum wait time in seconds
            
        Returns:
            bool: True if position reached, False if timeout
        """
        self.get_logger().info(f'Moving to ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})')
        
        start_time = time.time()
        while True:
            # Calculate distance to target
            distance = self.get_distance_to_target(target_x, target_y, target_z)
            
            # Check if reached (within tolerance)
            if distance < tolerance:
                self.get_logger().info(
                    f'Position reached: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})'
                )
                return True
            
            # Check timeout
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f'Position timeout. Distance: {distance:.2f}m'
                )
                return False
            
            # Update position feedback
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def land(self):
        """Switch to LAND mode"""
        self.get_logger().info('Landing...')
        return self.set_mode('LAND')
    
    def execute_mission(self):
        """
        Execute complete setpoint navigation mission.
        
        Flight plan:
        1. Takeoff to 10m
        2. Move to (10, 20, 10) - 10m East, 20m North
        3. Hold for 10 seconds
        4. Return to (0, 0, 10)
        5. Land
        """
        self.get_logger().info('=== Starting Setpoint Position Mission ===')
        
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
        self.get_logger().info('Waiting for altitude stabilization...')
        time.sleep(10)
        
        # Step 5: Move to position (10, 20, 10)
        self.send_position(10.0, 20.0, 10.0)
        if not self.wait_for_position(10.0, 20.0, 10.0):
            return False
        
        # Step 6: Hold position for 10 seconds
        self.get_logger().info('Holding position for 10 seconds...')
        time.sleep(10)
        
        # Step 7: Return to start (0, 0, 10)
        self.send_position(0.0, 0.0, 10.0)
        if not self.wait_for_position(0.0, 0.0, 10.0):
            return False
        
        # Step 8: Hold above takeoff point
        self.get_logger().info('Above takeoff point')
        time.sleep(3)
        
        # Step 9: Land
        if not self.land():
            return False
        
        self.get_logger().info('=== Mission Complete ===')
        return True


def main():
    """Main entry point"""
    rclpy.init()
    controller = SetpointController()
    
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
