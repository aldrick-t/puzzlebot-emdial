#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from your_package.msg import PoseWithVelocityTime
from nav_msgs.msg import Path
import math
import yaml

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        
        # Declare parameters
        self.declare_parameter('path_config', 'path_config.yaml')
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('max_linear_acceleration', 0.2)
        self.declare_parameter('max_angular_acceleration', 0.5)
        self.declare_parameter('robustness_factor', 1.2)  # Safety margin
        
        # Load path configuration
        config_file = self.get_parameter('path_config').value
        self.load_path_config(config_file)
        
        # Publisher for the path poses
        self.pose_pub = self.create_publisher(
            PoseWithVelocityTime, 
            '/pose', 
            10)
            
        # Publisher for visualization (optional)
        self.path_pub = self.create_publisher(
            Path,
            '/path',
            10)
            
        # Timer for publishing path points
        self.timer = self.create_timer(0.1, self.publish_next_pose)
        self.current_waypoint = 0
        
        self.get_logger().info("Path Generator Node has started!")
    
    def load_path_config(self, config_file):
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
                self.waypoints = config['waypoints']
                
                # Validate waypoints
                for i, wp in enumerate(self.waypoints):
                    if not self.validate_waypoint(wp):
                        self.get_logger().error(f"Waypoint {i} is invalid!")
                        raise ValueError(f"Invalid waypoint {i}")
                        
                self.get_logger().info(f"Loaded {len(self.waypoints)} valid waypoints")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load config file: {str(e)}")
            raise
    
    def validate_waypoint(self, waypoint):
        """Check if waypoint is reachable based on robot dynamics"""
        max_lin_vel = self.get_parameter('max_linear_velocity').value
        max_ang_vel = self.get_parameter('max_angular_velocity').value
        robustness = self.get_parameter('robustness_factor').value
        
        # Check if required velocities are within limits
        if 'linear_velocity' in waypoint and abs(waypoint['linear_velocity']) > max_lin_vel/robustness:
            return False
        if 'angular_velocity' in waypoint and abs(waypoint['angular_velocity']) > max_ang_vel/robustness:
            return False
            
        return True
    
    def calculate_missing_parameters(self, current_pose, waypoint):
        """Calculate missing parameters (time or velocities)"""
        # Extract current position
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        _, _, current_yaw = self.euler_from_quaternion(current_pose.orientation)
        
        # Extract target position
        target_x = waypoint['position']['x']
        target_y = waypoint['position']['y']
        target_yaw = waypoint.get('yaw', current_yaw)
        
        # Calculate distance and angle difference
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_diff = self.normalize_angle(target_yaw - current_yaw)
        
        # If time is given, calculate required velocities
        if 'time' in waypoint:
            time = waypoint['time']
            if time <= 0:
                raise ValueError("Time must be positive")
                
            waypoint['linear_velocity'] = distance / time
            waypoint['angular_velocity'] = angle_diff / time
            
        # If velocities are given, calculate required time
        elif 'linear_velocity' in waypoint and 'angular_velocity' in waypoint:
            lin_vel = waypoint['linear_velocity']
            ang_vel = waypoint['angular_velocity']
            
            if lin_vel == 0 and ang_vel == 0:
                raise ValueError("At least one velocity must be non-zero")
                
            time_lin = distance / lin_vel if lin_vel != 0 else float('inf')
            time_ang = angle_diff / ang_vel if ang_vel != 0 else float('inf')
            waypoint['time'] = max(time_lin, time_ang)
            
        else:
            raise ValueError("Waypoint must specify either time or both velocities")
            
        return waypoint
    
    def publish_next_pose(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("Finished publishing all waypoints")
            self.timer.cancel()
            return
            
        waypoint = self.waypoints[self.current_waypoint]
        msg = PoseWithVelocityTime()
        
        # Set position
        msg.pose.position.x = waypoint['position']['x']
        msg.pose.position.y = waypoint['position']['y']
        msg.pose.position.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        yaw = waypoint.get('yaw', 0.0)
        msg.pose.orientation = self.quaternion_from_euler(0, 0, yaw)
        
        # Calculate missing parameters if needed
        if 'time' not in waypoint or 'linear_velocity' not in waypoint or 'angular_velocity' not in waypoint:
            try:
                current_pose = self.get_last_pose()  # You'll need to implement this
                waypoint = self.calculate_missing_parameters(current_pose, waypoint)
            except Exception as e:
                self.get_logger().error(f"Error calculating parameters: {str(e)}")
                return
        
        # Set velocities and time
        msg.linear_velocity = waypoint['linear_velocity']
        msg.angular_velocity = waypoint['angular_velocity']
        msg.time_to_reach = waypoint['time']
        
        self.pose_pub.publish(msg)
        self.current_waypoint += 1
        
        self.get_logger().info(f"Published waypoint {self.current_waypoint}")
    
    # Helper functions
    def euler_from_quaternion(self, quaternion):
        """Convert quaternion to Euler angles"""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    path_generator = PathGenerator()
    rclpy.spin(path_generator)
    path_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()