from typing import Any
from random import random

import rclpy

# from .turtle import Turtle
from turtlesim.msg import Pose as TurtlePose
from turtle_control.control_law import ControlLaw

from std_srvs.srv import Trigger

to_sec = 1e-9

class RandomWalk(ControlLaw):
    timeout = 5.0
    def __init__(self, turtle):
        self.turtle = turtle
        self.goal_pose = self.random_goal()
        self.random_walk_service_topic = 'random_walk'
        
        self.spin_duration = 1/self.control_freq
        self.turtle.create_service(Trigger, self.random_walk_service_topic, self.execute, callback_group=self.turtle.service_server_cb_group)
        
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
        return self.goal_checker(self.turtle.current_pose, self.goal_pose)
        
    def execute(self, request:Trigger.Request, response:Trigger.Response):        
        self.goal_pose = self.random_goal() 
        self.turtle.get_logger().info('get random_goal request')
        
        try:
            while rclpy.ok() and self.turtle.context.ok():
                if self.is_goal_reach():
                    self.turtle.get_logger().info('goal_reach success')
                    response.success = True
                    response.message = 'random_goal reach'
                    return response
                elif self.is_timeout():
                    self.turtle.get_logger().info('timeout success')
                    response.success = True
                    response.message = 'timeout reach'
                    return response
            
                self.log()
                
                cmd_vel = self.go_to_pose(self.turtle.current_pose, self.goal_pose)
                
                self.turtle.pub_cmd_vel(cmd_vel)
                
                self.turtle.executor.spin_once(self.spin_duration)
        except Exception as e:
            self.turtle.get_logger().error(f"{e}")
            response.success = False
            response.message = 'runtime_exception'
            return response
            
    def log(self):
        curr = (self.turtle.current_pose.x, self.turtle.current_pose.y) 
        dist = (self.goal_pose.x, self.goal_pose.y)
        diff = self.pose_diff(self.turtle.current_pose, self.goal_pose)
        self.turtle.get_logger().debug('current_pose: ({:.1f},{:.1f}), goal_pose: ({:.1f},{:.1f}) pose_diff: ({:.1f},{:.1f})'.format(
            curr[0],curr[1],dist[0],dist[1],diff[0],diff[1]
        ))

        