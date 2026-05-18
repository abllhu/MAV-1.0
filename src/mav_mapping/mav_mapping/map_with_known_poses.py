#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener , Buffer , LookupException 
from tf_transformations import euler_from_quaternion

PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

class Pose:
    def __init__(self , px=0 , py=0):
        self.x = px
        self.y = py

def coordinatestopose(px , py , map_info:MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseonmap(pose:Pose , map_info:MapMetaData):
    return  pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

def bresenham(start: Pose, end: Pose):
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inversesensorModel(robot_postion : Pose , obstacle_pose : Pose):
    occ_values = []
    line = bresenham( robot_postion , obstacle_pose)
    for pose in line [:-1]:
        occ_values.append((pose,FREE_PROB))
    occ_values.append((line[-1] , OCC_PROB))
    return occ_values

def prob2logodds(p):
    return math.log(p/(1-p))


def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

class MapWithKnownPoses(Node):
    def __init__(self ,name):
        super().__init__( name)

        self.declare_parameter("width" , 50.0)
        self.declare_parameter("height" , 50.0)
        self.declare_parameter("resolution" , 0.1)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.origin.position.x = float(round(-width / 2.0))
        self.map_.info.origin.position.y = float(round(-height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.prob_map_ = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        self.map_pub_ = self.create_publisher(OccupancyGrid , "map" , 1)
        self.scan_sub_ = self.create_subscription(LaserScan , "scan" , self.scan_callback , 10)
        self.timer_ = self.create_timer(1.0 , self.timercallback)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_ , self)

    def scan_callback(self , msg:LaserScan):
        try:
            transform = self.tf_buffer_.lookup_transform(self.map_.header.frame_id , msg.header.frame_id , rclpy.time.Time())
            # Process the laser scan data and update the map
        except LookupException as e:
            self.get_logger().warn(f"Could not transform: {e}")
            return
        
        # Convert the robot's position from the transform to map coordinates
        robot_postion = coordinatestopose(transform.transform.translation.x , transform.transform.translation.y , self.map_.info)


        # Check if the robot's position is within the map bounds
        if not poseonmap(robot_postion , self.map_.info):
            self.get_logger().warn("Robot position is out of map bounds")
            return
        

        (roll , pitch , yaw) = euler_from_quaternion(
            [transform.transform.rotation.x , transform.transform.rotation.y,
            transform.transform.rotation.z , transform.transform.rotation.w])
        

        for i in range (len(msg.ranges)):
            if math.isinf(msg.ranges[i]):
                continue

            angle = msg.angle_min + (msg.angle_increment*i) + yaw
            px = msg.ranges[i] * math.cos(angle)
            py = msg.ranges[i] * math.sin(angle)
            px += transform.transform.translation.x
            py += transform.transform.translation.y

            obstacle_pose = coordinatestopose(px ,py , self.map_.info)
            if not poseonmap(obstacle_pose , self.map_.info):
                continue

            poses = inversesensorModel(robot_postion , obstacle_pose)
            for pose, value in poses :
                  cell = poseToCell(pose, self.map_.info)
                  self.prob_map_[cell]+= prob2logodds(value) - prob2logodds(PRIOR_PROB)
                  

    def timercallback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_.data = [int(logodds2prob(value)*100) for value in self.prob_map_]
        self.map_pub_.publish(self.map_)

def main(args=None):
    rclpy.init(args=args)
    node = MapWithKnownPoses("map_with_known_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


