"""
Author(s):
- ahmad-ahsan.bin-yusob@hs-coburg.de (Ahsan Yusob)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from mocap4r2_msgs.msg import RigidBodies, RigidBody
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from .transformation_util import (
    lin_2d_transform_g2b_quaternion, 
    lin_2d_transform_g2b_euler,
    adtf_compute_relative_quaternion,
    adtf_euler_from_quaternion,
    adtf_quaternion_from_euler,
)

from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException

import math
import time
import os

TIMER_PERIOD = 0.001  # LOOP PERIOD IN SEC
TIMEOUT      = 1.000  # TIMEOUT IN SEC
DEFAULT_CAR_ID = -1
ODOM_TOPIC   = 'odom_optitrack'
ODOM_QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
) #set QoS to reliable and transient local

############
# Speed Calculation Mode: 
#   1 - (WIP) Calculate the speed using vehicle positions in body coordinate.
#       This mode requires global2body transformation. 
#   2 - Calculate the speed using the magnitude of the position vectors. 
#       This mode uses global positions directly, 
#       assuming that v_x = (sqrt(dx² + dy²))/dt)) and that there's never v_y component due to slip to the side.
#############
SPEED_CALCULATION_MODE = 2
LPF_ALPHA = 0.2 # Low pass filter alpha (0-1). Lower is smoother.

class OdomOptitrack(Node):
    def __init__(self):
        super().__init__('odom_optitrack')
        self.publishers_ = dict()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('car_id', DEFAULT_CAR_ID, ParameterDescriptor(description="Set desired model car ID. If set to -1 (DEFAULT), ROS_DOMAIN_ID will be used.")),
                ('publish_tf', False,      ParameterDescriptor(description="map->carID/baselink is required for visualizing odom on map. Set to True, if it isn't available.")),
            ]
        )
        self.publish_tf=self.get_parameter("publish_tf").get_parameter_value().bool_value
        if self.publish_tf: self.tf_broadcaster = TransformBroadcaster(self)

        self.car_id=self.get_parameter("car_id").get_parameter_value().integer_value
        try: 
            self.car_id = int(os.environ["ROS_DOMAIN_ID"]) if self.car_id==-1 else self.car_id
        except:
            self.get_logger().error(f"Invalid CAR ID format. Got '{self.car_id}'.\n\tExpected integer or string-casted integer.. \n\tEither set the value with ros args 'car_id' or with ROS_DOMAIN_ID..")
            quit()

        self.rb_subscriber = self.create_subscription(RigidBodies,'/pose_modelcars', self.listener_callback_pose_modelcars, 1)
        self.timer = self.create_timer(TIMER_PERIOD,self.timer_callback)
        self.reset_topic_and_data()
        

    def reset_topic_and_data(self):
        """called to reset the odometry, if necessary"""
        curr_t = int(time.time_ns()*1e-6)
        self.prev_t = curr_t
        self.x_init_off = None
        self.y_init_off = None
        self.yaw_init_off = None
        self.prev_pose_baselink = None
        self.prev_pose_global = None
        self.filtered_vx = 0.0
        self.filtered_yawdot = 0.0

    def timer_callback(self):
        """check timer"""
        last_update = int(time.time_ns()*1e-6) - self.prev_t
        if last_update/1000 >= TIMEOUT:
            self.get_logger().info(f"Last update >={(last_update)/1000:.3f} sec ago. Resetting...")
            self.reset_topic_and_data()

    def listener_callback_pose_modelcars(self, msg:RigidBodies):
        curr_t = int(time.time_ns()*1e-6)
        for rigid_body in msg.rigidbodies:
            rb_:RigidBody = rigid_body
            
            #handle id
            try: id_ = int(rb_.rigid_body_name)
            except: self.get_logger().warn(f"Invalid rigid body name ignored (not string-casted integer e.g. '10')")
            if id_ != self.car_id: continue  #other id found in rigidbodies topic

            #handle pose
            pose_map:Pose = rb_.pose
            if (pose_map.position.x == pose_map.position.y == pose_map.position.z == 0.0): 
                self.get_logger().warn(f"selected car IDs {self.car_id} are out of optitrack vicinity (xyz-pos are all 0.0).")
                continue

            #publish tf "<car_id>/base_link" -> "map", when necessary
            parent_frame="map"
            child_frame=str(id_)+"/base_link"
            if self.publish_tf:
                self.make_transforms(
                    timestamp=msg.header.stamp,
                    parent=parent_frame,
                    child=child_frame,
                    position=pose_map.position,
                    orientation=pose_map.orientation
                )

            #compute odom
            if self.prev_pose_global != None:
                dt = (curr_t-self.prev_t)/1000
                if dt <= 0.001: continue #avoid capturing unnecessary dynamic
                odom = Odometry()
                odom.header.frame_id=parent_frame  #reference for pose
                odom.header.stamp=msg.header.stamp
                odom.child_frame_id=child_frame #reference for twist
                #assign pose (global pose w.r.t parent frame ("map" or "odom"))
                odom.pose.pose = pose_map
                #assign twist (twist w.r.t. child frame ("base_link" or "car_center"))
                roll_g, pitch_g, yaw_g = adtf_euler_from_quaternion(pose_map.orientation)
                if SPEED_CALCULATION_MODE==1:
                    pose_baselink = PoseWithCovariance()
                    #transform pose coord from global to body
                    pose_baselink.pose.position.x, pose_baselink.pose.position.y = lin_2d_transform_g2b_quaternion(pose_map.position.x, pose_map.position.y, yaw_g)
                    if self.x_init_off == self.y_init_off == self.yaw_init_off == None:
                        self.x_init_off = pose_baselink.pose.position.x
                        self.y_init_off = pose_baselink.pose.position.y
                        self.yaw_init_off = yaw_g
                    #apply init offset
                    pose_baselink.pose.position.x -= self.x_init_off
                    pose_baselink.pose.position.y -= self.y_init_off
                    pose_baselink.pose.orientation = adtf_quaternion_from_euler(0,0,yaw_g-self.yaw_init_off)
                    self.get_logger().debug(f"\n\t{pose_baselink.pose.position.x=:3.3f},\n\t{pose_baselink.pose.position.y=:3.3f},\n\t{pose_baselink.pose.orientation.z=:3.3f},\n\t{pose_baselink.pose.orientation.w=:3.3f}")
                    #calc translational speed
                    calculated_vx = (pose_baselink.pose.position.x - self.prev_pose_baselink.pose.position.x)/dt
                    calculated_vy = (pose_baselink.pose.position.y - self.prev_pose_baselink.pose.position.y)/dt
                    calculated_vz = (pose_baselink.pose.position.z - self.prev_pose_baselink.pose.position.z)/dt
                    #calc rotational speed
                    curr_yaw = yaw_g-self.yaw_init_off
                    prev_roll,prev_pitch,prev_yaw = adtf_euler_from_quaternion(self.prev_pose_baselink.pose.orientation)
                    calculated_yawdot = (curr_yaw - prev_yaw)/dt
                    #done
                    self.prev_pose_baselink = pose_baselink
                elif SPEED_CALCULATION_MODE==2:
                    #calc translational speed
                    calculated_vy = calculated_vz = 0.0
                    raw_vx = math.sqrt(
                        (pose_map.position.x - self.prev_pose_global.position.x)**2 +
                        (pose_map.position.y - self.prev_pose_global.position.y)**2
                    ) / dt
                    #apply low pass filter
                    self.filtered_vx = LPF_ALPHA * raw_vx + (1.0 - LPF_ALPHA) * self.filtered_vx
                    calculated_vx = self.filtered_vx
                    
                    #calc rotational speed
                    #use relative quaternion to avoid discontinuity of euler angles [-pi,pi]
                    q_rel = adtf_compute_relative_quaternion(self.prev_pose_global.orientation, pose_map.orientation)
                    dpitch,droll,dyaw = adtf_euler_from_quaternion(q_rel)
                    raw_yawdot = dyaw/dt
                    #apply low pass filter
                    self.filtered_yawdot = LPF_ALPHA * raw_yawdot + (1.0 - LPF_ALPHA) * self.filtered_yawdot
                    calculated_yawdot = self.filtered_yawdot

                odom.twist.twist.linear.x=calculated_vx
                odom.twist.twist.linear.y=calculated_vy
                odom.twist.twist.linear.z=calculated_vz
                odom.twist.twist.angular.z=calculated_yawdot
                self.get_logger().debug(f"\n\t{odom.twist.twist.linear.x=:3.3f},\n\t{odom.twist.twist.linear.y=:3.3f},\n\t{odom.twist.twist.angular.z=:3.3f}")

                #handle odom topic
                topic_ = '/modelcar'+rb_.rigid_body_name + '/'+ODOM_TOPIC
                if topic_ not in self.publishers_.keys():
                    self.publishers_.update({topic_: [self.create_publisher(Odometry,topic_,ODOM_QOS_PROFILE), curr_t]})
                    self.get_logger().info(f"Added new topic: {topic_}")

                #publish odom
                self.get_logger().debug(f"Publishing: {topic_}")
                self.publishers_[topic_][0].publish(odom)
            self.prev_pose_global = pose_map
        self.prev_t = curr_t

    def make_transforms(self, timestamp, parent:str, child:str, position:Point, orientation:Quaternion):
        t = TransformStamped()

        t.header.stamp = timestamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation = Vector3(x=position.x, y=position.y, z=position.z)
        t.transform.rotation = orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    try:
        rclpy.init(args=args)
        odom_optitrack = OdomOptitrack()
        rclpy.spin(odom_optitrack)
        odom_optitrack.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
    