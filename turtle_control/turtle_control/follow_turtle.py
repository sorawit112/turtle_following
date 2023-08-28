from typing import Any
from random import random
import numpy as np

import rclpy

from rclpy.node import Node

from turtlesim.msg import Pose as TurtlePose
from turtlesim_plus_interfaces.msg import ScannerData, ScannerDataArray
from turtle_control.control_law import ControlLaw

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from obo_ros_python_utils.custom_conversion import quaternion_from_euler

to_sec = 1e-9

class FollowTurtle(ControlLaw):
    timeout = 5.0
    offset_dist = 1.5
    def __init__(self, turtle:Node):
        self.turtle = turtle
        self.goal_pose = None
        self.seaching_service_topic = 'searching'
        self.following_service_topic = 'following'
        
        self.following = False
        
        self.spin_duration = 1/self.control_freq
        self.turtle.create_service(Trigger, self.seaching_service_topic, self.execute_searching, callback_group=self.turtle.service_server_cb_group)
        self.turtle.create_service(Trigger, self.following_service_topic, self.execute_following, callback_group=self.turtle.service_server_cb_group)

        self.timer = self.turtle.create_timer(self.spin_duration, self.timer_cb, self.turtle.timer_cb_group)
        
        self.goal_vis_pub = self.turtle.create_publisher(PoseStamped, '/goal_pose', qos_profile=1)
        
    def searching_turtle(self):
        if not self.turtle.scanner_array:
            return False
        
        for scan in self.turtle.scanner_array:
            scan:ScannerData
            if scan.type == 'Turtle' and abs(scan.angle) < np.deg2rad(10):
                self.goal_pose = self.cal_goal_pose(scan)
                self.turtle.scanner_array = None
                return True
            
        self.turtle.scanner_array = None
            
        return False
    
    def is_turtle_lost(self):
        if not self.turtle.scanner_array:
            return True
        
        for scan in self.turtle.scanner_array:
            scan:ScannerData
            if scan.type == 'Turtle':
                self.goal_pose = self.cal_goal_pose(scan)
                self.turtle.scanner_array = None
                return False
            
        self.turtle.scanner_array = None
            
        return True
    
    def is_goal_reach(self) -> bool:
        return self.goal_checker(self.turtle.current_pose, self.goal_pose)
    
    def cal_goal_pose(self, scan:ScannerData):
        pose = TurtlePose()
        current_pose = self.turtle.current_pose
        goal_dist = scan.distance-self.offset_dist
        angle = current_pose.theta + scan.angle
        pose.x = current_pose.x + np.cos(angle)*goal_dist
        pose.y = current_pose.y + np.sin(angle)*goal_dist
        pose.theta = current_pose.theta
        
        if self.goal_pose:
            self.pub_goal()
        
        return pose
    
    def pub_goal(self):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.turtle.get_clock().now().to_msg()
        pose.pose.position.x = self.goal_pose.x
        pose.pose.position.y = self.goal_pose.y
        qx,qy,qz,qw = quaternion_from_euler(0., 0., self.goal_pose.theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        self.goal_vis_pub.publish(pose)
    
    def reset_flag(self):
        self.following = False
        self.timer.destroy()
    
    async def timer_cb(self):
        if not self.following:
            return
        
        if self.is_goal_reach():
            self.turtle.stop()
        else:
            cmd_vel = self.go_to_pose(self.turtle.current_pose, self.goal_pose)
            self.turtle.pub_cmd_vel(cmd_vel)
            self.log('following')
        
        if self.is_turtle_lost():
            self.turtle.stop()
            self.turtle.get_logger().info("turtle lost")
            response = Trigger.Response()
            response.success = False
            response.message = 'turtle lost'
            
            self.following = False
            self.following_future.set_result(response)
    
    def execute_searching(self, request:Trigger.Request, response:Trigger.Response):
        self.turtle.get_logger().info('get seraching request')
        
        try:
            while rclpy.ok() and self.turtle.context.ok():
                if self.searching_turtle():
                    self.turtle.get_logger().info('found turtle')
                    response.success = True
                    response.message = 'found_turtle'
                    self.turtle.stop()
                    return response
                else:
                    self.turtle.scan_around()
                    self.log('scanning')
            
                    self.turtle.executor.spin_once(self.spin_duration)
                    # self.rate.sleep()
                    
        except Exception as e:
            self.turtle.get_logger().error(f"{e}")
            response.success = False
            response.message = 'runtime_exception'
            return response
        
        
    async def execute_following(self, request:Trigger.Request, response:Trigger.Response):
        self.turtle.get_logger().info('get following request')
        try:
            self.following_future = rclpy.Future()
            self.following = True
            
            return await self.following_future
    
        except Exception as e:
            self.turtle.get_logger().error(f"{e}")
            response.success = False
            response.message = 'runtime_exception'
            return response
            
    def log(self, state):
        if state == 'following':
            curr = (self.turtle.current_pose.x, self.turtle.current_pose.y) 
            dist = (self.goal_pose.x, self.goal_pose.y)
            diff = self.pose_diff(self.turtle.current_pose, self.goal_pose)
            self.turtle.get_logger().debug('{} : current_pose: ({:.1f},{:.1f}), goal_pose: ({:.1f},{:.1f}), pose_diff: ({:.1f},{:.1f})'.format(
                state, curr[0], curr[1], dist[0], dist[1], diff[0], diff[1]
            ))
        else:
            self.turtle.get_logger().debug(f'{state}')

        