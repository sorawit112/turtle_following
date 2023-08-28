from typing import List, Optional
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

from turtlesim_plus_interfaces.msg import ScannerData, ScannerDataArray
from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Twist

from rcl_interfaces.msg import SetParametersResult, Parameter

class ControlLaw():
    k_lin = 1.0
    k_ang = 2.8
    control_freq = 10.0
    goal_tolerance = 0.5
    max_k_lin = 3.0
    max_k_ang = 4.0
    def __init__(self, k_lin=None, k_ang=None):
        if k_lin:
            self.k_lin = k_lin
        if k_ang:
            self.k_ang = k_ang
    
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
        
        
class Turtle(Node):
    min_linear = 1.0
    max_linear = 3.0
    def __init__(self, k_lin=None, k_ang=None):
        super().__init__('node')
        self.cmd_vel_topic = "cmd_vel"
        self.scan_topic = "scan"
        self.pose_topic = "pose"
        
        self.declare_parameter('k_lin', k_lin)
        self.declare_parameter('k_ang', k_ang)
        
        k_lin = self.get_parameter('k_lin').value
        k_ang = self.get_parameter('k_ang').value
        
        self.scanner_array:Optional[ScannerDataArray] = None
        self.current_pose:Optional[TurtlePose] = None
        
        self.control = ControlLaw(k_lin, k_ang)
        
        self.subscriber_cb_group = ReentrantCallbackGroup()
        self.scan_sub = self.create_subscription(
                            ScannerDataArray, 
                            self.scan_topic,
                            self.scan_cb,
                            qos_profile=1, 
                            callback_group=self.subscriber_cb_group,
                        )
        self.pose_sub = self.create_subscription(
                            TurtlePose, 
                            self.pose_topic,
                            self.pose_cb,
                            qos_profile=1, 
                            callback_group=self.subscriber_cb_group,
                        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos_profile=1)
        self.control_rate = self.create_rate(self.control.control_freq)
        
        self.add_on_set_parameters_callback(self.set_control_gain)
         
    def scan_cb(self, msg:ScannerDataArray):
        self.scanner_array = msg.data
    
    def pose_cb(self, msg:TurtlePose):
        self.current_pose = msg
        
    def scan_around(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 2.0  
        
        self.pub_cmd_vel(cmd_vel)  
        
    def set_control_gain(self, params:List[Parameter]):
        self.get_logger().info('parameters callback')
        for param in params:
            if param.name == 'k_lin':
                if param.value >= 0:
                    if param.value > self.control.max_k_lin:
                        self.get_logger().warn(f'{param.value} greater than max_k_lin -> set to max_k_lin')
                    self.control.k_lin = min(param.value, self.control.max_k_lin)
                else:
                    self.get_logger().warn('k_lin value is negative -> skipping')
            elif param.name == 'k_ang':
                if param.value >= 0:
                    if param.value > self.control.max_k_ang:
                        self.get_logger().warn(f'{param.value} greater than max_k_ang -> set to max_k_ang')
                    self.control.k_ang = min(param.value, self.control.max_k_ang)
                else:
                    self.get_logger().warn('k_ang value is negative -> skipping')
            else:
                self.get_logger().warn(f"can't set {param.name}")
                 
        return SetParametersResult(successful=True)
        
    def stop(self):
        self.pub_cmd_vel(Twist())
    
    def pub_cmd_vel(self, cmd_vel:Twist):
        self.cmd_vel_pub.publish(self.clamped_vel(cmd_vel))    
        
    def clamped_vel(self, cmd_vel:Twist) -> Twist:
        clamped_cmd_vel = Twist()
        clamped_cmd_vel.linear.x = min(max(cmd_vel.linear.x, self.min_linear), self.max_linear)
        clamped_cmd_vel.angular.z = cmd_vel.angular.z
        
        return clamped_cmd_vel
        
def main(args=None):
    rclpy.init(args=args)
    
    turtle = Turtle()
    
    try:
        rclpy.spin(turtle)
    except:
        turtle.destroy_node()
    finally:
        try:
            rclpy.shutdown()
        except:
            print('rclpy already shutdown')

if __name__=="__main__":
    main()
    
    