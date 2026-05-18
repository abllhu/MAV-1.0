#!/usr/bin/env python3

import time
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray


# Robot States
class mav_state(Enum):
    NORMAL = 0
    WARNING = 1
    DANGER = 2


class SafetyStop(Node):

    def __init__(self):

        super().__init__('safety_stop')

        #  Parameters 
        # Distance for slowing down
        self.declare_parameter('warning_distance', 0.6)

        # Distance for full stop
        self.declare_parameter('danger_distance', 0.2)

        # Laser topic name
        self.declare_parameter('scan_topic', '/scan')

        # Safety stop topic name
        self.declare_parameter('safety_stop_topic', '/safety_stop')

        # Read parameters
        self.warning_distance = (
            self.get_parameter('warning_distance')
            .get_parameter_value()
            .double_value
        )

        self.danger_distance = (
            self.get_parameter('danger_distance')
            .get_parameter_value()
            .double_value
        )

        self.scan_topic = (
            self.get_parameter('scan_topic')
            .get_parameter_value()
            .string_value
        )

        self.safety_stop_topic = (
            self.get_parameter('safety_stop_topic')
            .get_parameter_value()
            .string_value
        )

        # Used only once to set marker frame_id
        self.is_first_msg = True

        # Current robot state
        self.state_ = mav_state.NORMAL

        # Previous robot state
        self.prev_state_ = mav_state.NORMAL

        #  Subscribers 
        # Subscribe to lidar scan
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scanCallback,
            10
        )

        # Publishers 
        # Publish emergency stop signal
        self.stop_pub_ = self.create_publisher(
            Bool,
            self.safety_stop_topic,
            10
        )

        # Publish RViz visualization markers
        self.zones_pub_ = self.create_publisher(
            MarkerArray,
            'zones',
            10
        )

        #  Action Clients 
        # Client for decreasing speed
        self.decrease_speed_client = ActionClient(
            self,
            JoyTurbo,
            'joy_turbo_decrease'
        )

        # Client for restoring speed
        self.increase_speed_client = ActionClient(
            self,
            JoyTurbo,
            'joy_turbo_increase'
        )

        # Wait until decrease action server is available
        while (
            not self.decrease_speed_client.wait_for_server(timeout_sec=1.0)
            and rclpy.ok()
        ):
            self.get_logger().warn(
                'Action /joy_turbo_decrease not available! Waiting..'
            )

            time.sleep(2.0)

        # Wait until increase action server is available
        while (
            not self.increase_speed_client.wait_for_server(timeout_sec=1.0)
            and rclpy.ok()
        ):
            self.get_logger().warn(
                'Action /joy_turbo_increase not available! Waiting..'
            )

            time.sleep(2.0)

        #RViz Zones 
        self.zones = MarkerArray()

        # Warning Zone (Yellow)
        warning_zone = Marker()

        warning_zone.id = 0
        warning_zone.type = Marker.CYLINDER
        warning_zone.action = Marker.ADD

        warning_zone.scale.z = 0.001
        warning_zone.scale.x = self.warning_distance * 2
        warning_zone.scale.y = self.warning_distance * 2

        warning_zone.color.r = 1.0
        warning_zone.color.g = 0.984
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.5

        # Danger Zone (Red)
        danger_zone = Marker()

        danger_zone.id = 1
        danger_zone.type = Marker.CYLINDER
        danger_zone.action = Marker.ADD

        danger_zone.scale.z = 0.001
        danger_zone.scale.x = self.danger_distance * 2
        danger_zone.scale.y = self.danger_distance * 2

        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.5

        # Raise red zone slightly above yellow zone
        danger_zone.pose.position.z = 0.01

        # Store both markers
        self.zones.markers = [warning_zone, danger_zone]


    # Lidar Callback
    def scanCallback(self, msg: LaserScan):

        # Default state
        self.state_ = mav_state.NORMAL

        # Check every lidar reading
        for range_value in msg.ranges:

            # Ignore inf values
            # inf means "no obstacle detected"
            if (
                not math.isinf(range_value)
                and range_value <= self.warning_distance
            ):

                # Obstacle inside warning zone
                self.state_ = mav_state.WARNING

                # Obstacle inside danger zone
                if range_value <= self.danger_distance:

                    self.state_ = mav_state.DANGER

                    # Stop checking remaining ranges
                    break

        # Execute only when state changes
        if self.state_ != self.prev_state_:

            # Bool message for emergency stop
            stop_msg = Bool()

            # ---------------- WARNING ----------------
            if self.state_ == mav_state.WARNING:

                # Do not stop robot
                stop_msg.data = False

                # Reduce robot speed
                self.decrease_speed_client.send_goal_async(
                    JoyTurbo.Goal()
                )

                # Highlight warning zone
                self.zones.markers[0].color.a = 1.0

                # Danger zone remains transparent
                self.zones.markers[1].color.a = 0.5

            # ---------------- DANGER ----------------
            elif self.state_ == mav_state.DANGER:

                # Emergency stop
                stop_msg.data = True

                # Highlight both zones
                self.zones.markers[0].color.a = 1.0
                self.zones.markers[1].color.a = 1.0

            # ---------------- NORMAL ----------------
            elif self.state_ == mav_state.NORMAL:

                # Robot can move normally
                stop_msg.data = False

                # Restore normal speed
                self.increase_speed_client.send_goal_async(
                    JoyTurbo.Goal()
                )

                # Make zones transparent again
                self.zones.markers[0].color.a = 0.5
                self.zones.markers[1].color.a = 0.5

            # Save current state
            self.prev_state_ = self.state_

            # Publish stop message
            self.stop_pub_.publish(stop_msg)

        # Set frame_id once
        if self.is_first_msg:

            for zone in self.zones.markers:

                # Use same frame as lidar
                zone.header.frame_id = msg.header.frame_id

            self.is_first_msg = False

        # Publish zones to RViz
        self.zones_pub_.publish(self.zones)


def main():

    rclpy.init()

    node = SafetyStop()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()