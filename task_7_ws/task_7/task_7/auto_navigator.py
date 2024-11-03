#!/usr/bin/env python3

import sys
import os
import numpy as np
from PIL import Image, ImageOps

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
import tf2_ros
from tf2_geometry_msgs import do_transform_point, do_transform_pose_stamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from scipy.spatial import KDTree

from geometry_msgs.msg import TransformStamped

class Navigation(Node):

    def __init__(self, node_name='Navigation'):

        super().__init__(node_name)
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.laser_data = None
        # Subscribers
        self.create_subscription(PoseStamped, '/robot/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/robot/amcl_pose', self.__ttbot_pose_cbk, 10)
        self.create_subscription(OccupancyGrid, '/robot/map', self.__map_cbk, 10)
        self.create_subscription(LaserScan, '/robot/scan', self.__laser_scan_cbk, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.look_ahead_index_pub = self.create_publisher(Float32, 'look_ahead_index', 10)
        self.look_ahead_error_pub = self.create_publisher(Float32, 'look_ahead_error', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_plan', 10)
        self.look_ahead_pose_pub = self.create_publisher(Marker, 'look_ahead_pose', 10)
        
        #tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        #map
        self.map_data = None
        self.map_origin = None
        self.map_resolution = None
        self.obstacle_direction = None
        
        self.new_target_received = False

        self.marker = self.initialize_marker()
        
        self.tree = None

        self.look_ahead_index = 6

        # Node rate
        self.timer = self.create_timer(0.05, self.run) 

        self.look_ahead_error = 0.0
        self.obstacle_near = False

    def initialize_marker(self):
        msg = Marker()
        msg.header.frame_id = 'base_link'
        msg.type = Marker.CUBE
        msg.action = Marker.ADD
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        return msg

    def __laser_scan_cbk(self, data):
        self.laser_data = data.ranges
        # self.get_logger().info(f'Laser data received: {len(self.laser_data)}')
        self.obstacle_near, self.obstacle_direction = self.find_obstacle(self.laser_data)

    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        self.new_target_received = True

    def __map_cbk(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        self.map_data = data
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_resolution = msg.info.resolution
        self.get_logger().info(f'Map received: {width}x{height}, origin: {self.map_origin}, resolution: {self.map_resolution}')
        obstacle_positions = np.argwhere(self.map_data == 100)
        # make more neighbors of the obstacle positions to make the robot avoid the obstacle
        self.get_logger().info(f'adding more neighbors to the obstacle positions')
        for pos in obstacle_positions:
            neighbor = self.get_more_neighbors((pos[1], pos[0]))
            if neighbor:
                for n in neighbor:
                    self.map_data[n[1], n[0]] = 100
        self.get_logger().info(f'map updated')

    def world_to_map(self, x, y):
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my

    def map_to_world(self, mx, my):
        x = mx * self.map_resolution + self.map_origin[0]
        y = my * self.map_resolution + self.map_origin[1]
        return x, y

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose

    def a_star_path_planner(self, start_pose, end_pose):
        path = Path()
        path.header.frame_id = 'map'
        if self.map_data is None:
            # self.get_logger().error("No map data available!")
            return path

        # Convert start and end poses to map coordinates
        start = self.world_to_map(start_pose.pose.position.x, start_pose.pose.position.y)
        goal = self.world_to_map(end_pose.pose.position.x, end_pose.pose.position.y)

        if start == goal:
            self.get_logger().info("Start and goal are the same!")
            return path
        self.get_logger().info(f'A* planner.\n> start: {start},\n> end: {goal}')

        open_list = [(self.heuristic(start, goal), start)]
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        came_from = {}          
        while open_list:
            open_list.sort(key=lambda x: x[0])
            _, current = open_list.pop(0)
            if current == goal:
                waypoints = self.reconstruct_path(came_from, current)
                for waypoint in waypoints:
                    pose = PoseStamped()
                    pose.pose.position.x, pose.pose.position.y = self.map_to_world(waypoint[0], waypoint[1])
                    path.poses.append(pose)
                path.poses.append(end_pose)
                self.get_logger().info("Path found!")
                return path
            for neighbor in self.get_neighbors(current):
                if self.map_data[neighbor[1], neighbor[0]] != 0:
                    continue
                tep_g_score = g_score[current] + self.distance(current, neighbor)
                if tep_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tep_g_score
                    f_score[neighbor] = tep_g_score + self.heuristic(neighbor, goal)
                    open_list.append((f_score[neighbor], neighbor))
        self.get_logger().error("No path found!")
        path.poses.append(start_pose)
        path.poses.append(end_pose)
        return path



    def heuristic(self, pos1, pos2):
        # Manhattan distance
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def get_neighbors(self, node):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.map_data.shape[1] and 0 <= ny < self.map_data.shape[0]:
                if self.map_data[ny, nx] == 0:  # Free space
                    neighbors.append((nx, ny))
        return neighbors

    def get_more_neighbors(self, node):
        buffer_range = 4  

        neighbors = []
        for i in range(-buffer_range, buffer_range + 1):
            for j in range(-buffer_range, buffer_range + 1):
                nx, ny = node[0] + i, node[1] + j

                # Check if the neighbor is within map boundaries
                if 0 <= nx < self.map_data.shape[1] and 0 <= ny < self.map_data.shape[0]:
                    # Add the neighbor if it's a free space (0)
                    if self.map_data[ny, nx] == 0:
                        neighbors.append((nx, ny))
        
        return neighbors

    def distance(self, pos1, pos2):
        return np.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()  # Reverse to get the path from start to goal
        return path

    def get_path_idx(self,vehicle_pose):
        if self.tree is None:
            return None
        x, y = vehicle_pose.pose.position.x, vehicle_pose.pose.position.y
        _, idx = self.tree.query((x, y))
        return idx
 
    def transform_path(self, path, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                path.header.frame_id,  
                rclpy.time.Time()
            )
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"Transform error: {e}")
            return None

        transformed_path = Path()
        transformed_path.header.frame_id = target_frame
        for pose in path.poses:
            pose.header.frame_id = path.header.frame_id
            transformed_pose = do_transform_pose_stamped(pose, transform)
            transformed_path.poses.append(transformed_pose)

        return transformed_path

    def quaternion_to_yaw(self, q):
        return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def silly_path_persuit(self, angle, distance):
        heading = 0.0
        speed = 0.0
        if angle > np.pi/8:
            heading = 0.25
            speed = 0.0
        elif angle < -np.pi/8:
            heading = -0.25
            speed = 0.0
        else:
            heading = min(0.25, max(-0.25, 2*angle))
            speed = min(0.1, 3*distance)
        return speed, heading
        
    def path_follower(self, closest_pose,target_pose, close_to_goal):
        if not target_pose:
            return
        if target_pose.pose.position.x < 0:
            self.get_logger().error("target behind the robot")
            heading = 0.4
            speed = 0.0
            return speed, heading
        if close_to_goal:
            angle = np.arctan2(target_pose.pose.position.y, target_pose.pose.position.x)
            distance = np.sqrt(target_pose.pose.position.x**2 + target_pose.pose.position.y**2)
            if distance < 0.02:
                self.get_logger().info("reached goal")
                speed = 0.0
                heading = 0.0
            else:
                return self.silly_path_persuit(angle, distance)
            return speed, heading
        else:
            angle = np.arctan2(target_pose.pose.position.y, target_pose.pose.position.x)
            distance = np.sqrt(target_pose.pose.position.x**2 + target_pose.pose.position.y**2)
            return self.silly_path_persuit(angle, distance)
            return speed, heading
        
        

    def move_ttbot(self, speed, heading):
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = heading
        self.cmd_vel_pub.publish(cmd_vel)

    def find_obstacle(self, laser_data):
        obstacle_directions = set()  # Use a set to store multiple directions
        if laser_data is None:
            return False, obstacle_directions

        threshold_distance = 0.1  # Minimum distance to consider an obstacle
        # Define ranges for each section (front, left, right, rear-left, rear-right)
        right_indices = range(240, 270)
        front_right_indices = range(270, 359 )  # 0
        front_left_indices = range(0, 90)  # 0
        left_indices = range(90, 120)
        for i, dist in enumerate(laser_data):
            if dist < threshold_distance:
                if i in right_indices:
                    obstacle_directions.add('right')
                elif i in front_right_indices:
                    obstacle_directions.add('front_right')
                elif i in front_left_indices:
                    obstacle_directions.add('front_left')
                elif i in left_indices:
                    obstacle_directions.add('left')

        if obstacle_directions:
            return True, obstacle_directions
        else:
            return False, obstacle_directions

    def wrap_to_pi_over_2(self, angle):
        while angle > np.pi/2:
            angle -= np.pi
        while angle < -np.pi/2:
            angle += np.pi
    def run(self):
        # print('map_data:', self.map_data)
        if self.new_target_received:
            self.move_ttbot(0.0, 0.0)
            self.get_logger().info("path generation started")
            self.new_target_received = False
            self.path = self.a_star_path_planner(self.ttbot_pose, self.goal_pose)
            self.tree = KDTree([(pose.pose.position.x, pose.pose.position.y) for pose in self.path.poses])
            self.get_logger().info("path generation completed")
        self.path_pub.publish(self.path)
        if not self.path.poses:
            self.get_logger().error("No valid path generated!")
            return
        
        closest_idx = self.get_path_idx(self.ttbot_pose)
        msg = Float32()
        msg.data = float(closest_idx)
        self.look_ahead_index_pub.publish(msg)
        remaining_path = Path()
        remaining_path.header.frame_id = 'map'
        if closest_idx < len(self.path.poses)-1:
            for i in range(closest_idx, len(self.path.poses)-1):
                remaining_path.poses.append(self.path.poses[i])
            local_path = self.transform_path(remaining_path, 'base_link')
        else:
            remaining_path.poses.append(self.path.poses[-1])
        if len(remaining_path.poses) > 20:
            remaining_path.poses = remaining_path.poses[:20]
        local_path = self.transform_path(remaining_path, 'base_link')
        self.local_path_pub.publish(local_path)
        close_to_goal = False
        if len(local_path.poses) > self.look_ahead_index+1:
            look_ahead_pose = local_path.poses[self.look_ahead_index]
        else:
            look_ahead_pose = local_path.poses[-1]
            close_to_goal = True
        self.marker.pose = look_ahead_pose.pose
        self.look_ahead_pose_pub.publish(self.marker)
        if self.obstacle_near:
            self.get_logger().info("obstacle close")
            self.move_ttbot(0.0, 0.0)
        else:
            speed, heading = self.path_follower(local_path.poses[0], look_ahead_pose, close_to_goal)
            self.move_ttbot(speed, heading)
       
def main(args=None):
    rclpy.init(args=args)
    nav = Navigation(node_name='Navigation')

    try:
        rclpy.spin(nav)
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()