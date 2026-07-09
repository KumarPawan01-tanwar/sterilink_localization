import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import subprocess, os

from mocap4r2_msgs.msg import RigidBodies, RigidBody
from geometry_msgs.msg import Pose, PoseWithCovariance
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.executors import ExternalShutdownException

DEFAULT_CAR_ID = -1
PKG_NAME = "ad_localiser"
ICP_LAUNCH_FILE = "laser_scan_matcher.launch.py"


class LocaliserManager(Node):
    def __init__(self):
        super().__init__('localiser_manager')
        #declare parameters
        #---------------
        # ODOM SELECTION
        #---------------
        # - bit0: odom_optitrack
        # - bit1: odom_icp
        # - bit2...: reserved
        # - e.g. 0=None; 1=odom_optitrack_only; 2=odom_icp_only; 3=Both;
        self.declare_parameters(
            namespace='',
            parameters=[
                ('car_id', DEFAULT_CAR_ID, ParameterDescriptor(description="Set desired model car ID. If set to -1 (DEFAULT), ROS_DOMAIN_ID will be used.")),
                ('tf_opti_absent', False,  ParameterDescriptor(description="map->carID/baselink is required for visualizing odom on map. Set to True, if it isn't available.")),
                ('odom_selection', 0b01,   ParameterDescriptor(description="Selection of odom: bit0=odom_optitrack; bit1=odom_icp; bit2..=reserved;")),  
            ]
        )
        self.car_id=self.get_parameter("car_id").get_parameter_value().integer_value
        self.tf_opti_absent=self.get_parameter("tf_opti_absent").get_parameter_value().bool_value  
        self.odom_selection=self.get_parameter("odom_selection").get_parameter_value().integer_value
        #handle parameters
        self.publish_tf_opti=(self.odom_selection&0b01==1 and self.tf_opti_absent)     #broadcast TF map->carID/baselink, if required AND absent.
        self.publish_tf_icp=(self.publish_tf_opti==False)                              #automatically disable broadcast TF odom->carID/base_link, if map->carID/baselink exists.
        try: 
            self.car_id = int(os.environ["ROS_DOMAIN_ID"]) if self.car_id==-1 else self.car_id
        except:
            self.get_logger().error(f"Invalid CAR ID format. Got '{self.car_id}'.\n\tExpected integer or string-casted integer.. \n\tEither set the value with ros args 'car_id' or with ROS_DOMAIN_ID..")
            quit()
        #declare subscriber
        self.rb_subscriber = self.create_subscription(RigidBodies,'/pose_modelcars', self.listener_callback_pose_modelcars, 1)
        #declare subprocess param
        self.process_optitrack = None
        self.process_icp = None
        #declare static transform publisher
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def listener_callback_pose_modelcars(self, msg:RigidBodies):

        if (self.process_optitrack==None) & (self.odom_selection&0b01):
            self.process_optitrack = subprocess.Popen(["ros2", "run", PKG_NAME, "odom_optitrack",
                                                       "--ros-args",
                                                       "-p", "car_id:="+str(self.car_id),
                                                       "-p", "publish_tf:="+str(self.publish_tf_opti)])

        if (self.process_icp==None) & (self.odom_selection&0b10)>>1:
            
            for rigid_body in msg.rigidbodies:
                rb_:RigidBody = rigid_body
                
                #handle id
                try: id_ = int(rb_.rigid_body_name)
                except: self.get_logger().warn(f"Invalid rigid body name ignored (not string-casted integer e.g. '10')")
                if id_ != self.car_id: continue  #other id found in rigidbodies topic

                #publish tf_static 
                pose_map:Pose = rb_.pose

                self.make_transforms(
                    timestamp=msg.header.stamp,
                    parent="map", 
                    child="odom", 
                    position=pose_map.position,
                    orientation=pose_map.orientation
                )

                #start ICP node
                # self.process_icp = subprocess.Popen(["ros2", "launch", PKG_NAME, ICP_LAUNCH_FILE])  
                # TODO: use launch; [/]figure out why the parameters seems to fail; [/]figure out whether it is possible to adapt some argument after launch;
                self.process_icp = subprocess.Popen(["ros2", "run", "ros2_laser_scan_matcher", "laser_scan_matcher", 
                                                     "--ros-args", 
                                                     "-p", "publish_odom:=odom", 
                                                     "-p", "publish_tf:="+str(self.publish_tf_icp), 
                                                     "-p", "laser_frame:=laser_frame", 
                                                     "-p", "base_frame:="+str(id_)+"/base_link"])
            

    def make_transforms(self, timestamp, parent:str, child:str, position:Point, orientation:Quaternion):
        t = TransformStamped()

        t.header.stamp = timestamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation = Vector3(x=position.x, y=position.y, z=position.z)
        t.transform.rotation = orientation

        self.tf_static_broadcaster.sendTransform(t)

    def shutdown_subprocesses(self):
        """Cleanly terminate the subprocesses."""
        if self.process_optitrack:
            self.get_logger().info("Terminating OptiTrack subprocess...")
            self.process_optitrack.terminate()
            self.process_optitrack.wait() # Ensure it's gone
            
        if self.process_icp:
            self.get_logger().info("Terminating ICP subprocess...")
            self.process_icp.terminate()
            self.process_icp.wait()
                

def main(args=None):
    try:
        rclpy.init(args=args)
        odom_icp = LocaliserManager()
        rclpy.spin(odom_icp)
        odom_icp.destroy_node()
        odom_icp.shutdown_subprocesses()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
