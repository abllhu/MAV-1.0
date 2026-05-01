#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from scipy.spatial.transform import Rotation


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")

        self.wr_ = 0.033 #wheel_radius
        self.ws_ = 0.17 #wheel_separation

        self.get_logger().info(f"Using wheel radius {self.wr_}")
        self.get_logger().info(f"Using wheel separation {self.ws_}")

        self.right_wheel_prev_pos_ = 0.0
        self.left_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Subscribe in the simuulated encoder readings for the wheels
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointcallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "mav_controller/odom_noisy", 10)

        self.speed_conversion_ = np.array([[self.wr_/2, self.wr_/2],
                                           [self.wr_/self.ws_, -self.wr_/self.ws_]])
        self.get_logger().info(f"The conversion matrix is {self.speed_conversion_}")


        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.broadcast_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"


    def jointcallback(self , msg):
        # The inverse differential kinematics model to calculate V , W 
        # Given the position of the wheels and the time between two readings
        # From V , W we can get wheel_odom(postion , oriantation) 
        wheel_right_encoder = msg.position[0] + np.random.normal(0, 0.005) # Add noise to the encoder readings
        wheel_left_encoder = msg.position[1] + np.random.normal(0, 0.005) # Add noise to the encoder readings
        dp_left = wheel_left_encoder - self.left_wheel_prev_pos_
        dp_right = wheel_right_encoder - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # Actualize the prev pose for the next itheration
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Calculate the rotational speed of each wheel
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        # Calculate the linear and angular velocity
        linear = (self.wr_ * fi_right + self.wr_ * fi_left) / 2
        angular = (self.wr_ * fi_right - self.wr_ * fi_left) / self.ws_

        # By integrating the linear and angular velocity we can get the position and orientation of the robot
        d_s = (self.wr_ * dp_right + self.wr_ * dp_left) / 2
        d_theta = (self.wr_ * dp_right - self.wr_ * dp_left) / self.ws_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = Rotation.from_euler('z', self.theta_).as_quat()
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        self.broadcast_.sendTransform(self.transform_stamped_)



def main():
    rclpy.init()

    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)

    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()