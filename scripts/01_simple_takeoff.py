#!/usr/bin/env python3

"""
Simple Takeoff Example
======================
This script demonstrates the basic autonomous flight sequence:
1. Connect to MAVROS
2. Change to GUIDED mode
3. Arm the motors
4. Takeoff to 15 meters
5. Hover for 10 seconds
6. Land automatically

Author: SimToFly
License: MIT
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Header
import time


class SimpleTakeoffController(Node):
    """
    Simple takeoff and landing controller using MAVROS services.
    """
    
    def __init__(self):
        super().__init__('simple_takeoff_controller')
        
        # State variables
        self.current_state = None
        self.connected = False
        self.armed = False
        self.current_mode = ""
        
        # Subscribe to drone state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # Create service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        self.get_logger().info('Simple Takeoff Controller initialized')
        
        # Wait for services to become available
        self.wait_for_services()
    
    def state_callback(self, msg):
        """
        Callback for /mavros/state topic.
        Updates current drone state information.
        """
        self.current_state = msg
        self.connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode
    
    def wait_for_services(self):
        """
        Wait for all MAVROS services to become available.
        This ensures the flight controller is ready before sending commands.
        """
        self.get_logger().info('Waiting for MAVROS services...')
        
        services = [
            (self.arming_client, 'arming'),
            (self.set_mode_client, 'set_mode'),
            (self.takeoff_client, 'takeoff')
        ]
        
        for client, name in services:
            client.wait_for_service()
            self.get_logger().info(f'Service {name} available')
        
        self.get_logger().info('All services ready')
    
    def wait_for_connection(self, timeout=30):
        """
        Wait for MAVROS to connect to the flight controller.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            bool: True if connected, False if timeout
        """
        self.get_logger().info('Waiting for MAVROS connection...')
        
        start_time = time.time()
        while not self.connected:
            if time.time() - start_time > timeout:
                self.get_logger().error('Connection timeout!')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Connected to MAVROS')
        return True
    
    def set_mode(self, mode, timeout=10):
        """
        Change flight mode.
        
        Args:
            mode: Desired flight mode (e.g., 'GUIDED', 'STABILIZE')
            timeout: Maximum time to wait for mode change in seconds
            
        Returns:
            bool: True if mode changed successfully
        """
        self.get_logger().info(f'Changing to {mode} mode...')
        
        # Create mode change request
        request = SetMode.Request()
        request.custom_mode = mode
        
        # Call service
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().mode_sent:
            self.get_logger().error(f'Failed to set {mode} mode')
            return False
        
        # Wait for mode to actually change
        start_time = time.time()
        while self.current_mode != mode:
            if time.time() - start_time > timeout:
                self.get_logger().error('Mode change timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info(f'Mode changed to {mode}')
        return True
    
    def arm(self, timeout=10):
        """
        Arm the drone motors.
        
        Args:
            timeout: Maximum time to wait for arming in seconds
            
        Returns:
            bool: True if armed successfully
        """
        self.get_logger().info('Arming motors...')
        
        # Create arming request
        request = CommandBool.Request()
        request.value = True
        
        # Call arming service
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().success:
            self.get_logger().error('Arming command rejected')
            return False
        
        # Wait for armed status to change
        start_time = time.time()
        while not self.armed:
            if time.time() - start_time > timeout:
                self.get_logger().error('Arming timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Motors armed successfully')
        return True
    
    def takeoff(self, altitude, timeout=30):
        """
        Command the drone to takeoff to specified altitude.
        
        Args:
            altitude: Target altitude in meters (relative to takeoff point)
            timeout: Maximum time to wait for takeoff command acceptance
            
        Returns:
            bool: True if takeoff command accepted
        """
        self.get_logger().info(f'Taking off to {altitude}m...')
        
        # Create takeoff request
        request = CommandTOL.Request()
        request.altitude = altitude
        request.latitude = 0.0   # Not used for local takeoff
        request.longitude = 0.0  # Not used for local takeoff
        request.min_pitch = 0.0
        request.yaw = 0.0
        
        # Call takeoff service
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if not future.result().success:
            self.get_logger().error('Takeoff command rejected')
            return False
        
        self.get_logger().info('Takeoff command sent')
        return True
    
    def hover(self, duration):
        """
        Hover in place for specified duration.
        
        Args:
            duration: Hover time in seconds
        """
        self.get_logger().info(f'Hovering for {duration} seconds...')
        time.sleep(duration)
        self.get_logger().info('Hover complete')
    
    def land(self):
        """
        Command the drone to land at current position.
        Landing mode will automatically disarm after touchdown.
        
        Returns:
            bool: True if land mode set successfully
        """
        self.get_logger().info('Landing...')
        
        # Switch to LAND mode
        # ArduPilot will handle descent and automatic disarm
        success = self.set_mode('LAND')
        
        if success:
            self.get_logger().info('Land mode activated')
        else:
            self.get_logger().error('Failed to activate land mode')
        
        return success
    
    def execute_mission(self):
        """
        Execute the complete takeoff, hover, and landing sequence.
        
        Returns:
            bool: True if mission completed successfully
        """
        self.get_logger().info('=== Starting Simple Takeoff Mission ===')
        
        # Step 1: Wait for connection
        if not self.wait_for_connection():
            return False
        
        # Step 2: Change to GUIDED mode
        if not self.set_mode('GUIDED'):
            return False
        
        # Step 3: Arm motors
        if not self.arm():
            return False
        
        # Step 4: Takeoff to 15 meters
        if not self.takeoff(altitude=15.0):
            return False
        
        # Wait for takeoff to complete (altitude stabilization)
        self.get_logger().info('Waiting for altitude stabilization...')
        time.sleep(10)
        
        # Step 5: Hover for 10 seconds
        self.hover(duration=10)
        
        # Step 6: Land
        if not self.land():
            return False
        
        self.get_logger().info('=== Mission Complete ===')
        return True


def main():
    """
    Main function to initialize ROS2 and execute the mission.
    """
    # Initialize ROS2
    rclpy.init()
    
    # Create controller node
    controller = SimpleTakeoffController()
    
    try:
        # Execute the mission
        success = controller.execute_mission()
        
        if success:
            controller.get_logger().info('Mission succeeded!')
        else:
            controller.get_logger().error('Mission failed!')
        
        # Keep node alive to see landing complete
        controller.get_logger().info('Waiting for landing to complete...')
        time.sleep(15)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Mission interrupted by user')
    except Exception as e:
        controller.get_logger().error(f'Mission error: {str(e)}')
    finally:
        # Cleanup
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
