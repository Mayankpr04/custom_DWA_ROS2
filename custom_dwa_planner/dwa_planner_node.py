#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from collections import deque

class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')

        param_names = [
            'max_vel', 'min_vel', 'max_yawrate', 'max_accel', 'max_dyawrate', 'vel_res',
            'yawrate_res', 'dt', 'predict_time', 'heading_cost_gain', 'speed_cost_gain', 
            'obstacle_cost_gain'
        ]

        self.params = {}
        for name in param_names:
            if not self.has_parameter(name):
                self.declare_parameter(name, None)
            self.params[name] = self.get_parameter(name).value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)     # publishers and subscribers
        self.traj_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.goal_pub = self.create_publisher(Marker, '/dwa_goal', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("DWA Planner node initialized")

        self.current_pose = None
        self.current_vel = {'v': 0.0, 'omega': 0.0}
        self.scan_data = None
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.01749
        
        self.start_pose = None
        self.goal_absolute = None       # pass goal point in odom frame, placeholder
        self.goal_reached = False
        
        self.position_history = deque(maxlen=15)  # for stall check, incase needed
        self.stuck_threshold = 0.05  # 5 cm is the threshold for being stuck
        self.recovery_mode = False
        self.recovery_timer = 0

        self.timer = self.create_timer(self.params['dt'], self.control_loop)
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_vel['v'] = msg.twist.twist.linear.x
        self.current_vel['omega'] = msg.twist.twist.angular.z
    
    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment
    
    def is_stuck(self):
        if len(self.position_history) < 30:
            return False
        oldest_pos = self.position_history[0]
        current_pos = self.position_history[-1]
        
        displacement = np.hypot(
            current_pos[0] - oldest_pos[0],
            current_pos[1] - oldest_pos[1]
        )
        return displacement < self.stuck_threshold
    
    def is_goal_reached(self, goal, tolerance=0.15):        # 15cm tolerance to goal point
        x, y, _ = self.pose_to_xytheta(self.current_pose)
        distance = np.hypot(goal[0] - x, goal[1] - y)
        return distance < tolerance
    
    def dynamic_window(self):
        p = self.params
        vs = [p['min_vel'], p['max_vel'], -p['max_yawrate'], p['max_yawrate']]
        vd = [self.current_vel['v'] - p['max_accel'] * p['dt'],
              self.current_vel['v'] + p['max_accel'] * p['dt'],
              self.current_vel['omega'] - p['max_dyawrate'] * p['dt'],
              self.current_vel['omega'] + p['max_dyawrate'] * p['dt']]
        dw = [max(vs[0], vd[0]),
              min(vs[1], vd[1]),
              max(vs[2], vd[2]),
              min(vs[3], vd[3])]
        return dw
    
    def pose_to_xytheta(self, pose):
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y**2 + q.z**2)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        return x, y, theta
    
    def calc_heading_cost(self, x, y, theta, goal):
        dx = goal[0] - x
        dy = goal[1] - y
        angle_to_goal = np.arctan2(dy, dx)
        diff = angle_to_goal - theta
        diff = np.arctan2(np.sin(diff), np.cos(diff))
        return 1.0 - (abs(diff) / np.pi)
    
    def calc_obstacle_cost(self, trajectory):
        if self.scan_data is None:
            return 0.0
        x_r, y_r, theta_r = self.pose_to_xytheta(self.current_pose)
        obstacles = []
        for i, r in enumerate(self.scan_data):
            if np.isinf(r) or np.isnan(r) or r > 20.0:
                continue
            angle = i * self.scan_angle_increment       # lidar orientation
            ox = x_r + r*np.cos(angle + theta_r)        # frame transform to odom
            oy = y_r + r*np.sin(angle + theta_r)        
            obstacles.append((ox, oy))
        if not obstacles:
            return 0.0
        min_dist = float('inf')
        for (x, y) in trajectory:
            for (ox, oy) in obstacles:
                dist = np.hypot(ox-x, oy-y)
                if dist < min_dist:
                    min_dist = dist
        robot_radius = 0.10
        safety_margin = 0.05        # 5cm safety margin, can tweak if needed 
        collision_threshold = robot_radius + safety_margin  
        if min_dist < collision_threshold:
            return float('inf')     # reject collision trajectories
        max_cost = 10.0
        cost = max_cost * np.exp(-2.0 * (min_dist - collision_threshold))   # exponential cost decay
        return min(cost, max_cost)

    def simple_recovery(self):
        self.recovery_timer += 1
        twist = Twist()
        twist.linear.x = -0.1
        twist.angular.z = 0.0  
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Recovery: Backing up ({self.recovery_timer}/20)", 
                              throttle_duration_sec=0.5)
        
        if self.recovery_timer >= 20:
            self.recovery_mode = False
            self.recovery_timer = 0
            self.position_history.clear()  # Reset stuck detection
            self.get_logger().info("Recovery complete")

    def control_loop(self):
        if self.current_pose is None or self.scan_data is None:
            return
        x, y, theta = self.pose_to_xytheta(self.current_pose)
        self.position_history.append((x, y))
        if self.start_pose is None:
            self.start_pose = (x, y, theta)
            self.goal_absolute = np.array([1.26, 1.64])  # set goal point in odom frame
            self.get_logger().info(f"Start: ({x:.2f}, {y:.2f}), Goal: {self.goal_absolute})")
        goal = self.goal_absolute
        if self.is_goal_reached(goal, tolerance=0.15):
            if not self.goal_reached:
                self.get_logger().info("GOAL REACHED!")
                self.goal_reached = True
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.publish_goal_marker(goal)
            return
        
        if len(self.position_history) >= 30 and not self.recovery_mode:
            if self.is_stuck():
                self.get_logger().warn("Robot is stuck (no movement), entering recovery")
                self.recovery_mode = True
                self.recovery_timer = 0
        if self.recovery_mode:
            self.simple_recovery()
            self.publish_goal_marker(goal)
            return
        
        marker_array = MarkerArray()
        dw = self.dynamic_window()      # resume dynamic window calculation
        
        if dw[1]-dw[0] < self.params['vel_res']:
            dw[1] = dw[0] + self.params['vel_res']
        if dw[3]-dw[2] < self.params['yawrate_res']:
            dw[3] = dw[2] + self.params['yawrate_res']
        
        best_score = -float('inf')
        best_v, best_omega = 0.0, 0.0
        marker_id = 0
        
        x_start, y_start, theta_start = x, y, theta
        for v in np.arange(dw[0], dw[1], self.params['vel_res']):
            for omega in np.arange(dw[2], dw[3], self.params['yawrate_res']):
                x_traj, y_traj, theta_traj = x_start, y_start, theta_start
                trajectory = []
                for t in np.arange(0, self.params['predict_time'], self.params['dt']):
                    x_traj += v * np.cos(theta_traj) * self.params['dt']
                    y_traj += v * np.sin(theta_traj) * self.params['dt']
                    theta_traj += omega * self.params['dt']
                    trajectory.append((x_traj, y_traj))
                
                marker = Marker()       # visualize trajectory
                marker.header.frame_id = 'odom'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'dwa_trajectories'
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.01
                marker.color.a = 0.3
                marker.color.r = 0.7
                marker.color.g = 0.7
                marker.color.b = 0.7
                marker.points = [Point(x=px, y=py, z=0.0) for (px, py) in trajectory]
                marker_array.markers.append(marker)
        
                heading_cost = self.calc_heading_cost(x_traj, y_traj, theta_traj, goal)
                speed_cost = v / self.params['max_vel']
                obstacle_cost = self.calc_obstacle_cost(trajectory)
                if obstacle_cost == float('inf'):
                    continue
                
                score = (self.params['heading_cost_gain'] * heading_cost +
                        self.params['speed_cost_gain'] * speed_cost -
                        self.params['obstacle_cost_gain'] * obstacle_cost)
                
                if score > best_score:
                    best_score = score
                    best_v, best_omega = v, omega

        if best_score == -float('inf'):
            self.get_logger().warn(" No valid trajectory - entering recovery")
            self.recovery_mode = True
            self.recovery_timer = 0
            twist = Twist()
            self.cmd_pub.publish(twist)
            return
        
        best_omega = np.clip(best_omega, -3.0, 3.0)
        twist = Twist()
        twist.linear.x = best_v
        twist.angular.z = best_omega
        self.cmd_pub.publish(twist)

        x_traj, y_traj, theta_traj = x_start, y_start, theta_start      # visualize best trajectory
        best_traj_points = []
        for t in np.arange(0, self.params['predict_time'], self.params['dt']):
            x_traj += best_v * np.cos(theta_traj) * self.params['dt']
            y_traj += best_v * np.sin(theta_traj) * self.params['dt']
            theta_traj += best_omega * self.params['dt']
            best_traj_points.append(Point(x=x_traj, y=y_traj, z=0.0))
        
        best_marker = Marker()
        best_marker.header.frame_id = 'odom'
        best_marker.header.stamp = self.get_clock().now().to_msg()
        best_marker.ns = 'best_trajectory'
        best_marker.id = 9999
        best_marker.type = Marker.LINE_STRIP
        best_marker.action = Marker.ADD
        best_marker.scale.x = 0.05
        best_marker.color.a = 1.0
        best_marker.color.r = 0.0
        best_marker.color.g = 1.0
        best_marker.color.b = 0.0
        best_marker.points = best_traj_points
        marker_array.markers.append(best_marker)
        
        self.traj_pub.publish(marker_array)
        self.publish_goal_marker(goal)
        distance_to_goal = np.hypot(goal[0] - x_start, goal[1] - y_start)
        self.get_logger().info(
            f"v={best_v:.2f}, omega={best_omega:.2f}, score={best_score:.2f}, distance to goal={distance_to_goal:.2f}m",
            throttle_duration_sec=1.0
        )
    
    def publish_goal_marker(self, goal):
        goal_marker = Marker()
        goal_marker.header.frame_id = 'odom'
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'dwa_goal'
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = goal[0]
        goal_marker.pose.position.y = goal[1]
        goal_marker.pose.position.z = 0.0
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        goal_marker.color.a = 1.0
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        self.goal_pub.publish(goal_marker)

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
