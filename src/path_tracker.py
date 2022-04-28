
import math
import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
import sys


import Astar, plotting
from geometry_msgs.msg  import Twist, PoseStamped
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion


def angle_calculator(current_location, next_point):
    """ Calculate angle between two points. """
    theta = math.atan2((next_point[1] - current_location[1]), (next_point[0] - current_location[0]))
    return theta


def difference_current_goal(current_yaw, goal_yaw):
    """ Calculate the difference between the current orientation and the orientation in which its supposed to be."""
    buffer = 5  # if it's close enough, don't turn
    turn = goal_yaw - current_yaw
    if -buffer < turn < buffer:
        turn = 0
    return turn


class PathTracker(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.cnt = 0
        self.astar = Astar.AStar("euclidean")
        self.path_coordinates = []
        self.goal_points = []
        self.start_points = []
        self.image_sub = self.create_subscription(Image, '/obstacle_map', self.image_sub_callback, 10)
        self.optitrack = self.create_subscription(PoseStamped, '/optitrack', self.optitrack_sub_callback, 10)  # create subscription for optitrack system
        self.motor_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw = 0  # to test system without optitrack

    def move_jetbot(self, turn, speed=20, angular_speed=5):
        """ Move the jetbot. """
        vel = Twist()
        if turn == 0:
            print('Going forward')
            vel.linear.x = speed
            vel.angular.z = 0.0
        else:
            print('Turning')
            vel.linear.x = 0.0
            if turn < 0:
                angular_speed = -angular_speed
            vel.angular.z = angular_speed
        # publish to /cmd_vel
        self.motor_publisher.publish(vel)

    def image_sub_callback(self, msg):
        """
        Use A* algorithm for path planning. Get coordinate list for path tracking
        """
        self.astar.obs, self.goal_points, self.start_points, dim = self.astar.Env.obs_map(msg)
        self.astar.s_start = self.start_points[0]
        self.astar.s_goal = np.array((500,125))
        plot = plotting.Plotting(self.astar.s_start, self.astar.s_goal,
                                 self.astar.obs, self.goal_points, self.start_points, dim)
        
        self.path_coordinates, visited = self.astar.searching()

        print(self.path_coordinates)
        plot.animation(self.path_coordinates, visited, "A*")


    def update_current_state(self, current_coordinates, current_direction):
        """ Update current state using optitrack system. """
        next_point = np.array(self.path_coordinates[self.cnt + 1])

        position_difference = next_point - current_coordinates

        angle = angle_calculator(current_coordinates, next_point)
        self.yaw = angle  # to test system without optitrack
        turn = difference_current_goal(current_direction, angle)
        move_jetbot(turn)

        if position_difference < 10:  # if it is close enough, take the next way point
            self.cnt += 1

    def optitrack_sub_callback(self, msg):
        """ Callback for optitrack system. """
        # TODO convert optitrack coordinates to obstacle map or the other way around
        # x, y = msg.Pose.Position.X, msg.Pose.Position.Y
        # yaw = euler_from_quaternion(msg.Pose.Orientation)[-1]
        # current_coordinates = [x, y]
        # current_direction = yaw
        if len(self.path_coordinates) == 0:  # path is not calculated yet
            pass
        else:
            # if sum(abs(self.astar.s_goal - current_coordinates)) < 10:
            # reached goal, TODO stop
            current_coordinates = self.path_coordinates[self.cnt]  # to test system without optitrack
            current_direction = self.yaw  # to test system without optitrack
            self.update_current_state(current_coordinates, current_direction)


def main():
    rclpy.init()
    path_tracker = PathTracker()

    # Spin until ctrl + c
    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
