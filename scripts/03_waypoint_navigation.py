#!/usr/bin/env python3

"""
Waypoint Mission Navigation Example
====================================
Correct sequence:
1. GUIDED mode
2. Arm motors
3. Takeoff to altitude
4. Upload waypoints
5. Switch to AUTO mode
6. Execute waypoint mission

Author: Sidharth Mohan Nair https://github.com/sidharthmohannair | SimToFly https://github.com/simtofly
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, Waypoint, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix
import time


class WaypointMissionController(Node):
    
    def __init__(self):
        super().__init__('waypoint_mission_controller')
        
        # State variables
        self.connected = False
        self.armed = False
        self.current_mode = ""
        
        # Home position
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.home_received = False
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)
        
        # Subscribe to mission reached
        self.mission_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.mission_callback,
            qos_profile
        )
        
        # Mission tracking
        self.last_waypoint_reached = 0
        self.total_waypoints = 0
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.waypoint_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        
        self.get_logger().info('Controller initialized')
        self.wait_for_services()
    
    def state_callback(self, msg):
        self.connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode
    
    def gps_callback(self, msg):
        if not self.home_received:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.home_received = True
            self.get_logger().info(f'Home: ({self.home_lat:.6f}, {self.home_lon:.6f}, {self.home_alt:.1f}m)')
    
    def mission_callback(self, msg):
        """Monitor mission progress"""
        self.last_waypoint_reached = msg.wp_seq
        self.get_logger().info(f'Reached waypoint {msg.wp_seq}/{self.total_waypoints}')
    
    def wait_for_services(self):
        self.get_logger().info('Waiting for services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.waypoint_push_client.wait_for_service()
        self.waypoint_clear_client.wait_for_service()
        self.get_logger().info('Services ready')
    
    def wait_for_connection(self, timeout=30):
        self.get_logger().info('Waiting for connection...')
        start_time = time.time()
        while not self.connected:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Connected')
        return True
    
    def wait_for_home_position(self, timeout=30):
        start_time = time.time()
        while not self.home_received:
            if time.time() - start_time > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True
    
    def set_mode(self, mode, timeout=10):
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
        self.get_logger().info(f'Takeoff to {altitude}m')
        request = CommandTOL.Request()
        request.altitude = altitude
        
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().success:
            return False
        
        self.get_logger().info('Takeoff command sent')
        return True
    
    def read_waypoints(self, filename):
        waypoints = []
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split(',')
                    if len(parts) != 3:
                        continue
                    lat = float(parts[0].strip())
                    lon = float(parts[1].strip())
                    alt = float(parts[2].strip())
                    waypoints.append((lat, lon, alt))
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
            return waypoints
        except Exception as e:
            self.get_logger().error(f'Failed to read waypoints: {str(e)}')
            return []
    
    def upload_mission(self, waypoint_list):
        self.get_logger().info('Uploading mission...')
        
        # Clear mission
        clear_req = WaypointClear.Request()
        future = self.waypoint_clear_client.call_async(clear_req)
        rclpy.spin_until_future_complete(self, future)
        
        # Create waypoints
        waypoints = []
        
        # Home waypoint
        wp = Waypoint()
        wp.frame = 0
        wp.command = 16
        wp.is_current = True
        wp.autocontinue = True
        wp.param1 = 0.0
        wp.param2 = 0.0
        wp.param3 = 0.0
        wp.param4 = 0.0
        wp.x_lat = self.home_lat
        wp.y_long = self.home_lon
        wp.z_alt = self.home_alt
        waypoints.append(wp)
        
        # Navigation waypoints
        for i, (lat, lon, alt) in enumerate(waypoint_list):
            wp = Waypoint()
            wp.frame = 3
            wp.command = 16
            wp.is_current = False
            wp.autocontinue = True
            wp.param1 = 0.0
            wp.param2 = 2.0
            wp.param3 = 0.0
            wp.param4 = 0.0
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            waypoints.append(wp)
            self.get_logger().info(f'  WP{i+1}: ({lat:.6f}, {lon:.6f}, {alt}m)')
        
        # Upload
        push_req = WaypointPush.Request()
        push_req.start_index = 0
        push_req.waypoints = waypoints
        
        future = self.waypoint_push_client.call_async(push_req)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().success:
            self.get_logger().error('Upload failed')
            return False
        
        self.get_logger().info(f'Mission uploaded: {len(waypoints)} waypoints')
        return True
    
    def execute_mission(self, waypoint_file):
        self.get_logger().info('=== Starting Mission ===')
        
        # Connect
        if not self.wait_for_connection():
            return False
        
        if not self.wait_for_home_position():
            return False
        
        # Read waypoints
        waypoints = self.read_waypoints(waypoint_file)
        if not waypoints:
            return False
        
        # Track total waypoints (home + navigation waypoints)
        self.total_waypoints = len(waypoints) + 1
        
        takeoff_alt = waypoints[0][2]
        
        # GUIDED mode
        if not self.set_mode('GUIDED'):
            return False
        
        # Arm
        if not self.arm():
            return False
        
        # Takeoff
        if not self.takeoff(takeoff_alt):
            return False
        
        # Wait for altitude
        self.get_logger().info('Reaching altitude...')
        time.sleep(15)
        
        # Upload waypoints
        if not self.upload_mission(waypoints):
            return False
        
        # AUTO mode
        if not self.set_mode('AUTO'):
            return False
        
        self.get_logger().info('Mission executing...')
        
        # Monitor mission progress
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # Check if last waypoint reached
            if self.last_waypoint_reached >= self.total_waypoints - 1:
                self.get_logger().info('Mission complete - switching to RTL')
                self.set_mode('RTL')
                break
        
        return True


def main():
    rclpy.init()
    controller = WaypointMissionController()
    
    try:
        success = controller.execute_mission('data/waypoints.txt')
        if success:
            controller.get_logger().info('Mission completed successfully')
        else:
            controller.get_logger().error('Mission failed')
    except KeyboardInterrupt:
        controller.get_logger().info('Stopped')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
