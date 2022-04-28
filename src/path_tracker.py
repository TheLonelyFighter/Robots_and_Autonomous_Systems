
import math
import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Image

import Astar, plotting


def angle_calculator(current_location, next_point):
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
        self.image_sub = self.create_subscription(Image, 'Obstacle_map', self.image_sub_callback, 10)
        #self.optitrack = self.create_subscription()  # create subscription for optitrack system
        self.motor_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.astar = Astar.AStar("euclidean")
        self.path_coordinates = []
        self.goal_points = []
        self.start_points = [] 

    def move_jetbot(self, turn, speed=20, angular_speed=5):
        vel = Twist()
        if turn == 0:
            vel.linear.x = speed
            vel.angular.z = 0.0
        else:
            vel.linear.x = 0.0
            vel.angular.z = angular_speed
        # publish to /cmd_vel
        self.motor_publisher.publish(vel)

    def image_sub_callback(self, msg):
        """
        Use A* algorithm for path planning. Get coordinate list for path tracking
        """
        self.astar.obs, self.goal_points, self.start_points, dim = self.astar.Env.obs_map(msg)
        self.astar.s_start = self.start_points[0]
        self.astar.s_goal = (500,125)
        plot = plotting.Plotting(self.astar.s_start, self.astar.s_goal,
                                 self.astar.obs, self.goal_points, self.start_points, dim)
        
        self.path_coordinates, visited, = self.astar.searching()

        print(self.path_coordinates)
        plot.animation(self.path_coordinates, visited, "A*")



    def update_current_state(self):
        """ Update current state using optitrack system. """
        # get current yaw and current position
        angle_calculator()
        difference_current_goal()
        move_jetbot()


def main():
    rclpy.init()
    path_tracker = PathTracker()

    # Spin until ctrl + c
    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
