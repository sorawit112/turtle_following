from typing import Any
from random import random
import numpy as np

import rclpy

from turtle_control.turtle import Turtle
from turtlesim.msg import Pose as TurtlePose
from turtlesim_plus_interfaces.msg import ScannerData, ScannerDataArray

to_sec = 1e-9

class FollowTurtle:
    timeout = 5.0
    offset_dist = 1.5
    def __init__(self, turtle:Turtle):
        self.turtle = turtle
        self.goal_pose = None
        self.turtle.create_timer(1/self.turtle.control.control_freq, self.execute)    
    
    def searching(self):
        if not self.turtle.scanner_array:
            return False
        for scan in self.turtle.scanner_array:
            scan:ScannerData
            if scan.type == 'Turtle':
                self.goal_pose = self.cal_goal_pose(scan)
                self.turtle.scanner_array = None
                return True
            
        self.turtle.scanner_array = None
            
        return False
    
    def is_goal_reach(self) -> bool:
        return self.turtle.control.goal_checker(self.turtle.current_pose, self.goal_pose)
    
    def cal_goal_pose(self, scan:ScannerData):
        pose = TurtlePose()
        current_pose = self.turtle.current_pose
        goal_dist = scan.distance-self.offset_dist
        angle = current_pose.theta + scan.angle
        pose.x = current_pose.x + np.cos(angle)*goal_dist
        pose.y = current_pose.y + np.sin(angle)*goal_dist
        pose.theta = scan.angle
        
        return pose
            
    def execute(self):       
        if self.searching():
            if self.is_goal_reach():
                self.turtle.stop()
            else:
                cmd_vel = self.turtle.control.go_to_pose(self.turtle.current_pose, self.goal_pose)
                self.turtle.pub_cmd_vel(cmd_vel)
            self.log('following')
        else:
            self.turtle.scan_around()
            self.log('scanning')
            
    def log(self, state):
        if state == 'following':
            curr = (self.turtle.current_pose.x, self.turtle.current_pose.y) 
            dist = (self.goal_pose.x, self.goal_pose.y)
            diff = self.turtle.control.pose_diff(self.turtle.current_pose, self.goal_pose)
            self.turtle.get_logger().debug('{} : current_pose: ({:.1f},{:.1f}), goal_pose: ({:.1f},{:.1f}), pose_diff: ({:.1f},{:.1f})'.format(
                state, curr[0], curr[1], dist[0], dist[1], diff[0], diff[1]
            ))
        else:
            self.turtle.get_logger().debug(f'{state}')
        
def main(args=None):
    rclpy.init(args=args)
    k_lin = 2.0
    k_ang = 2.0
    turtle = Turtle(k_lin, k_ang)
    follow_turtle = FollowTurtle(turtle)
    try:
        rclpy.spin(turtle)
    except Exception as e:
        print(e)
        turtle.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

        