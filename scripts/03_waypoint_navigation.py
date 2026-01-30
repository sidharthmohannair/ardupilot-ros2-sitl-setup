#!/usr/bin/env python3

"""
Waypoint Mission Navigation Example
====================================
This script demonstrates GPS-based waypoint mission execution:
1. Read waypoints from file (latitude, longitude, altitude)
2. Upload mission to ArduPilot
3. Takeoff and switch to AUTO mode
4. Execute waypoint mission
5. Land at final waypoint

Waypoints use global GPS coordinates (latitude/longitude).

Author: SimToFly
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, Waypoint, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix
import time


class WaypointMissionController(Node):
    """
    GPS waypoint mission controller using MAVROS mission services.
    """
    
    def __init__(self):
        super().__init__('waypoint_mission_controller')
        
        # State variables
        self.connected = False
        self.armed = False
        self.current_mode = ""
        
        # Home position (GPS)
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.home_received = False
        
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
        
        # Subscribe to GPS position to get home location
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.waypoint_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        
        self.get_logger().info('Waypoint Mission Controller initialized')
        self.wait_for_services()
    
    def state_callback(self, msg):
        """Update state"""
        self.connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode
    
    def gps_callback(self, msg):
        """
        Receive current GPS position.
        First position is stored as home location.
        """
        if not self.home_received:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.home_received = True
            self.get_logger().info(
                f'Home position: ({self.home_lat:.6f}, {self.home_lon:.6f}, {self.home_alt:.1f}m)'
            )
    
    def wait_for_services(self):
        """Wait for mission services"""
        self.get_logger().info('Waiting for services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.waypoint_push_client.wait_for_service()
        self.waypoint_clear_client.wait_for_service()
        self.get_logger().info('Services ready')
    
    def wait_for_connection(self, timeout=30):
        """Wait for MAVROS connection"""
        self.get_logger().info('Waiting for connection...')
        start_time = time.time()
        while not self.connected:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Connected')
        return True
    
    def wait_for_home_position(self, timeout=30):
        """Wait for GPS home position"""
        self.get_logger().info('Waiting for home position...')
        start_time = time.time()
        while not self.home_received:
            if time.time() - start_time > timeout:
                self.get_logger().error('Home position timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
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
    
    def read_waypoints_from_file(self, filename):
        """
        Read waypoints from text file.
        
        File format (one waypoint per line):
        latitude, longitude, altitude
        
        Lines starting with # are comments.
        
        Args:
            filename: Path to waypoints file
            
        Returns:
            list: List of (lat, lon, alt) tuples
        """
        waypoints = []
        
        try:
            with open(filename, 'r') as f:
                for line in f:
                    # Skip comments and empty lines
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse waypoint
                    parts = line.split(',')
                    if len(parts) != 3:
                        continue
                    
                    lat = float(parts[0].strip())
                    lon = float(parts[1].strip())
                    alt = float(parts[2].strip())
                    
                    waypoints.append((lat, lon, alt))
            
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {filename}')
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f'Failed to read waypoints: {str(e)}')
            return []
    
    def create_waypoint(self, frame, command, is_current, autocontinue, 
                       param1, param2, param3, param4, 
                       x_lat, y_lon, z_alt):
        """
        Create a MAVLink waypoint message.
        
        This follows the MAVLink waypoint protocol format.
        """
        wp = Waypoint()
        wp.frame = frame
        wp.command = command
        wp.is_current = is_current
        wp.autocontinue = autocontinue
        wp.param1 = param1
        wp.param2 = param2
        wp.param3 = param3
        wp.param4 = param4
        wp.x_lat = x_lat
        wp.y_long = y_lon
        wp.z_alt = z_alt
        return wp
    
    def upload_mission(self, waypoint_list):
        """
        Upload waypoint mission to ArduPilot.
        
        Args:
            waypoint_list: List of (lat, lon, alt) tuples
            
        Returns:
            bool: True if upload successful
        """
        self.get_logger().info('Uploading mission...')
        
        # Clear existing mission first
        clear_req = WaypointClear.Request()
        future = self.waypoint_clear_client.call_async(clear_req)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().success:
            self.get_logger().error('Failed to clear mission')
            return False
        
        # Create mission waypoints
        waypoints = []
        
        # Waypoint 0: Home position (required by ArduPilot)
        # Command 16 = MAV_CMD_NAV_WAYPOINT
        home_wp = self.create_waypoint(
            frame=0,           # Global frame
            command=16,        # NAV_WAYPOINT
            is_current=True,   # This is current waypoint
            autocontinue=True,
            param1=0.0,        # Hold time (seconds)
            param2=0.0,        # Acceptance radius (meters)
            param3=0.0,        # Pass radius
            param4=0.0,        # Yaw angle
            x_lat=self.home_lat,
            y_lon=self.home_lon,
            z_alt=self.home_alt
        )
        waypoints.append(home_wp)
        
        # Add mission waypoints
        for i, (lat, lon, alt) in enumerate(waypoint_list):
            wp = self.create_waypoint(
                frame=3,           # Global frame relative to home altitude
                command=16,        # NAV_WAYPOINT
                is_current=False,
                autocontinue=True,
                param1=0.0,        # Hold time
                param2=2.0,        # Acceptance radius (2m)
                param3=0.0,
                param4=0.0,        # Yaw (0 = don't change)
                x_lat=lat,
                y_lon=lon,
                z_alt=alt          # Relative altitude
            )
            waypoints.append(wp)
            self.get_logger().info(f'  WP{i+1}: ({lat:.6f}, {lon:.6f}, {alt}m)')
        
        # Push waypoints to FCU
        push_req = WaypointPush.Request()
        push_req.start_index = 0
        push_req.waypoints = waypoints
        
        future = self.waypoint_push_client.call_async(push_req)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().success:
            self.get_logger().error('Mission upload failed')
            return False
        
        self.get_logger().info(f'Mission uploaded: {len(waypoints)} waypoints')
        return True
    
    def execute_mission(self, waypoint_file):
        """
        Execute complete waypoint mission.
        
        Args:
            waypoint_file: Path to waypoints.txt file
        """
        self.get_logger().info('=== Starting Waypoint Mission ===')
        
        # Step 1: Connect
        if not self.wait_for_connection():
            return False
        
        # Step 2: Wait for home position
        if not self.wait_for_home_position():
            return False
        
        # Step 3: Read waypoints
        waypoints = self.read_waypoints_from_file(waypoint_file)
        if not waypoints:
            self.get_logger().error('No waypoints loaded')
            return False
        
        # Step 4: Upload mission
        if not self.upload_mission(waypoints):
            return False
        
        # Step 5: Set GUIDED mode and arm
        if not self.set_mode('GUIDED'):
            return False
        
        if not self.arm():
            return False
        
        # Wait for stabilization
        time.sleep(3)
        
        # Step 6: Switch to AUTO mode to execute mission
        if not self.set_mode('AUTO'):
            return False
        
        self.get_logger().info('Mission executing...')
        self.get_logger().info('Monitor progress in MAVProxy console')
        self.get_logger().info('Mission will complete automatically')
        
        # Keep node alive while mission executes
        # Mission duration depends on waypoint count and distances
        self.get_logger().info('Press Ctrl+C to stop monitoring')
        
        return True


def main():
    """Main entry point"""
    rclpy.init()
    controller = WaypointMissionController()
    
    # Waypoint file path
    waypoint_file = 'data/waypoints.txt'
    
    try:
        success = controller.execute_mission(waypoint_file)
        
        if success:
            controller.get_logger().info('Mission started successfully')
            # Keep running to monitor mission
            rclpy.spin(controller)
        else:
            controller.get_logger().error('Mission failed to start')
        
    except KeyboardInterrupt:
        controller.get_logger().info('Monitoring stopped')
    except Exception as e:
        controller.get_logger().error(f'Error: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
