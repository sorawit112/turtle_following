from typing import List, Optional
import numpy as np

from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Twist
        

class ControlLaw:
    k_lin = 1.0
    k_ang = 2.8
    control_freq = 10.0
    goal_tolerance = 1.5
    def __init__(self, k_lin=None, k_ang=None):
        if k_lin:
            self.k_lin = k_lin
        if k_ang:
            self.k_ang = k_ang
            
    def set_linear_gain(self, gain):
        self.k_lin = gain
        
    def set_angular_gain(self, gain):
        self.k_ang = gain
    
    def pose_diff(self, current_pose:TurtlePose, goal_pose:TurtlePose) -> np.array:
        dx = goal_pose.x - current_pose.x
        dy = goal_pose.y - current_pose.y
        
        return np.array([dx, dy])
    
    def theta_goal(self, current_pose:TurtlePose, goal_pose:TurtlePose):
        diff = self.pose_diff(current_pose, goal_pose)        
        
        return np.arctan2(diff[1],diff[0])
    
    def go_to_pose(self, current_pose:TurtlePose, goal_pose:TurtlePose) -> Twist:
        diff = self.pose_diff(current_pose, goal_pose)        

        err_dist = np.linalg.norm(diff)
        err_theta = self.theta_goal(current_pose, goal_pose) - current_pose.theta
        
        cmd_vel = Twist()
        cmd_vel.linear.x = self.k_lin*err_dist
        cmd_vel.angular.z = self.k_ang*np.arctan2(np.sin(err_theta),np.cos(err_theta))
        
        return cmd_vel        
    
    def goal_checker(self, current_pose:TurtlePose, goal_pose:TurtlePose) -> bool:
        diff = self.pose_diff(current_pose, goal_pose)        

        return np.linalg.norm(diff) <= self.goal_tolerance