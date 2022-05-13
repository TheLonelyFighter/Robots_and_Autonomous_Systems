
import math
import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node
import sys

import time
import Astar, plotting
from geometry_msgs.msg  import Twist, PoseStamped
from sensor_msgs.msg import Image
from move_drone import imgmsg_to_cv2


def angle_calculator(current_location, next_point):
    """ Calculate angle between two points. """
    theta = math.atan2((next_point[1] - current_location[1]), (next_point[0] - current_location[0]))
    return theta


def difference_current_goal(current_yaw, goal_yaw):
    """ Calculate the difference between the current orientation and the orientation in which its supposed to be."""
    buffer = 0.3 # if it's close enough, don't turn
    turn = goal_yaw - current_yaw
    if -buffer < turn < buffer:
        turn = 0
    return turn

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PathTracker(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.pos_cnt = 0
        self.astar = Astar.AStar("euclidean")
        self.current_coordinates = ()
        self.current_direction = 0
        self.path_coordinates = []
        self.goal_point = (None,None)
        self.start_points = []
        self.got_map = False
        self.reach_goal = False
        self.image_sub = self.create_subscription(Image, '/tello_map_image', self.image_sub_callback, 10)
        self.optitrack = self.create_subscription(PoseStamped, '/vrpn_client_node/jetbot146/pose', self.optitrack_sub_callback, 10)  # create subscription for optitrack system
        self.motor_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_jetbot(self, turn, speed=0.25, angular_speed=0.44):
        """ Move the jetbot. """
        vel = Twist()
        if turn == 0:
            # print('Going forward')
            vel.linear.x = speed
            vel.angular.z = 0.0
        else:
            # print('Turning')
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
        if not self.got_map:
            self.got_map = True
            self.astar.obs, self.goal_point = self.astar.Env.obs_map(msg)
            self.astar.s_start = self.current_coordinates
            self.astar.s_goal = (self.goal_point[0], self.goal_point[1])             

            plot = plotting.Plotting(self.astar.s_start, self.astar.s_goal,
                                    self.astar.obs, self.goal_point, self.start_points, (350,350))
            
            self.path_coordinates, visited = self.astar.searching()

            self.path_coordinates.reverse() #The path list starts from the goal and goes to the start, so we reverse the list
            print(self.path_coordinates)
            plot.animation(self.path_coordinates, visited, "A*")


    def update_current_state(self, current_coordinates, current_direction):
        """ Update current state using optitrack system. """
        if not self.reach_goal:
            if(self.pos_cnt + 1 >= len(self.path_coordinates)):
                #Reached end, stop
                self.move_jetbot(turn=0, speed=0.0, angular_speed=0.0)
                self.reach_goal = True
            else:
                next_point = np.array(self.path_coordinates[self.pos_cnt + 1])

                position_difference = next_point - current_coordinates
                # print(f'position_difference: {position_difference}.')
                angle = angle_calculator(current_coordinates, next_point)
                # print("angle in rad= ", angle)
                #print("angle in degree = ", np.rad2deg(angle))
                # print(f'current_coordinates = {current_coordinates}, next_point = {next_point}.')
                turn = difference_current_goal(current_direction, angle)
                self.move_jetbot(turn)

                if abs(position_difference[0]) < 20 and abs(position_difference[1]) < 20:  # if it is close enough, take the next way point
                    self.pos_cnt += 1

    def optitrack_sub_callback(self, msg):
        """ Callback for optitrack system. """

        if len(self.path_coordinates) == 0:  # path is not calculated yet
            pass
            #print("No path yet")
            self.current_coordinates = (int(msg.pose.position.x * 100), int(msg.pose.position.y * 100))
            self.current_direction = euler_from_quaternion(msg.pose.orientation)[-1]
        else:
            self.current_coordinates = (int(msg.pose.position.x * 100), int(msg.pose.position.y * 100))
            self.current_direction = euler_from_quaternion(msg.pose.orientation)[-1]

            # print(f'current_direction = {self.current_direction}')
            self.update_current_state(self.current_coordinates, self.current_direction)

def main():
    rclpy.init()
    path_tracker = PathTracker()
    print("Path tracker initialized, waiting for image (path).")
    # Spin until ctrl + c
    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
