from typing import List, Optional
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

from turtlesim_plus_interfaces.msg import ScannerData, ScannerDataArray
from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Twist, TransformStamped

from rcl_interfaces.msg import SetParametersResult, Parameter

from turtle_control.follow_turtle import FollowTurtle
from turtle_control.random_walk import RandomWalk

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from obo_ros_python_utils.custom_conversion import quaternion_from_euler
        
        
class Turtle(Node):
    min_linear = 1.0
    max_linear = 3.0
    max_k_lin = 3.0
    max_k_ang = 10.0
    min_angle = 0.3
    
    def __init__(self, k_lin=2.0, k_ang=2.0):
        super().__init__('node')
        self.cmd_vel_topic = "cmd_vel"
        self.scan_topic = "scan"
        self.pose_topic = "pose"
        
        self.declare_parameter('k_lin', k_lin)
        self.declare_parameter('k_ang', k_ang)
        
        self.scanner_array:Optional[ScannerDataArray] = None
        self.current_pose:Optional[TurtlePose] = None
        self.subscriber_cb_group = ReentrantCallbackGroup()
        self.service_server_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()
        
        self.tf_pub = TransformBroadcaster(self, qos=1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
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
        
        self.add_on_set_parameters_callback(self.set_control_gain)
        
        self.follow_turtle = FollowTurtle(self)
        self.random_walk = RandomWalk(self)
         
    def scan_cb(self, msg:ScannerDataArray):
        self.scanner_array = msg.data
    
    def pose_cb(self, msg:TurtlePose):
        self.current_pose = msg
        self.publish_tf(msg)
    
    def publish_tf(self, msg:TurtlePose):
        tf_stamped = TransformStamped()
        tf_stamped.header.frame_id = 'world'
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.child_frame_id = self.get_namespace()+'/pose'
        tf_stamped.transform.translation.x = msg.x
        tf_stamped.transform.translation.y = msg.y
        qx,qy,qz,qw = quaternion_from_euler(0., 0., msg.theta)
        tf_stamped.transform.rotation.x = qx
        tf_stamped.transform.rotation.y = qy
        tf_stamped.transform.rotation.z = qz
        tf_stamped.transform.rotation.w = qw
        
        self.tf_pub.sendTransform(tf_stamped)
        
    def scan_around(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 2.0  
        
        self.pub_cmd_vel(cmd_vel)  
        
    def set_control_gain(self, params:List[Parameter]):
        self.get_logger().info('parameters callback')
        for param in params:
            if param.name == 'k_lin':
                if param.value >= 0:
                    if param.value > self.max_k_lin:
                        self.get_logger().warn(f'{param.value} greater than max_k_lin -> set to max_k_lin')
                    self.follow_turtle.set_linear_gain(min(param.value, self.max_k_lin))
                    self.random_walk.set_linear_gain(min(param.value, self.max_k_lin))
                else:
                    self.get_logger().warn('k_lin value is negative -> skipping')
            elif param.name == 'k_ang':
                if param.value >= 0:
                    if param.value > self.max_k_ang:
                        self.get_logger().warn(f'{param.value} greater than max_k_ang -> set to max_k_ang')
                    self.follow_turtle.set_angular_gain(min(param.value, self.max_k_ang))
                    self.random_walk.set_angular_gain(min(param.value, self.max_k_ang))
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
        clamped_cmd_vel.angular.z = max(abs(cmd_vel.angular.z), self.min_angle) * np.sign(cmd_vel.angular.z)
        
        return clamped_cmd_vel
        
def main(args=None):
    rclpy.init(args=args)
    
    turtle = Turtle()
    
    try:
        rclpy.spin(turtle)
    except Exception as e:
        print(e)
        turtle.destroy_node()
    finally:
        try:
            rclpy.shutdown()
        except:
            print('rclpy already shutdown')

if __name__=="__main__":
    main()
    
    