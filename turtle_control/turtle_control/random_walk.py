from typing import Any
from random import random

import rclpy

from turtle_control.turtle import Turtle
from turtlesim.msg import Pose as TurtlePose

to_sec = 1e-9

class RandomWalk:
    timeout = 5.0
    def __init__(self, turtle:Turtle):
        self.turtle = turtle
        self.goal_pose = self.random_goal()
        self.turtle.create_timer(1/self.turtle.control.control_freq, self.execute)
        
    def random_goal(self) -> TurtlePose:
        rand_x = random()*10.0
        rand_y = random()*10.0
        rand_theta = (random()-0.5)*6.28
                
        goal_pose = TurtlePose()
        goal_pose.x = rand_x
        goal_pose.y = rand_y
        goal_pose.theta = rand_theta        
        self.start_time = self.turtle.get_clock().now()
        
        return goal_pose
    
    def is_timeout(self):
        return (self.turtle.get_clock().now()-self.start_time).nanoseconds*to_sec > self.timeout
    
    def is_goal_reach(self) -> bool:
        return self.turtle.control.goal_checker(self.turtle.current_pose, self.goal_pose)
        
    def execute(self):               
        if self.is_goal_reach() or self.is_timeout():
            self.goal_pose = self.random_goal() 
            
        self.log()
        
        cmd_vel = self.turtle.control.go_to_pose(self.turtle.current_pose, self.goal_pose)
        
        self.turtle.pub_cmd_vel(cmd_vel)
            
    def log(self):
        curr = (self.turtle.current_pose.x, self.turtle.current_pose.y) 
        dist = (self.goal_pose.x, self.goal_pose.y)
        diff = self.turtle.control.pose_diff(self.turtle.current_pose, self.goal_pose)
        self.turtle.get_logger().debug('current_pose: ({:.1f},{:.1f}), goal_pose: ({:.1f},{:.1f}) pose_diff: ({:.1f},{:.1f})'.format(
            curr[0],curr[1],dist[0],dist[1],diff[0],diff[1]
        ))
        
def main(args=None):
    rclpy.init(args=args)
    k_lin = 0.6
    k_ang = 2.0
    turtle = Turtle(k_lin, k_ang)
    random_walk = RandomWalk(turtle)
    try:
        rclpy.spin(turtle)
    except Exception as e:
        print(e)
        turtle.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

        