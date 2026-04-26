"""
Author(s): 
- ahmad-ahsan.bin-yusob@hs-coburg.de (Ahsan Yusob)
"""

from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_inverse
import math
import numpy as np
import time
from geometry_msgs.msg import Quaternion


def adtf_euler_from_quaternion(quat, qformat='ros', to_deg=False):
    """
    Read quaternion in different formats
    - 'ros' : geometry_msgs/Quaternion
    - 'xyzw': tuple(qx,qy,qz,qw)
    - 'wxyz': tuple(qw,qx,qy,qz)\n
    and\n

    Return `roll, pitch, yaw in radians` (or in degrees if `to_deg==True`).
    
    Debug: return `-1` for invalid format.\n
    """
    if qformat=='ros': quat:Quaternion=quat; qx=quat.x; qy=quat.y; qz=quat.z; qw=quat.w
    elif qformat=='xyzw': qx,qy,qz,qw=quat
    elif qformat=='wxyz': qw,qx,qy,qz=quat
    else: raise Exception  #TODO: raise proper exception
    roll,pitch,yaw=euler_from_quaternion((qx,qy,qz,qw))
    if to_deg: roll,pitch,yaw=[math.degrees(rad_) for rad_ in (roll,pitch,yaw)]
    return roll,pitch,yaw


def adtf_quaternion_from_euler(roll, pitch, yaw, qformat='ros', in_deg=False):
    """
    Read euler angles in radians (or in degrees if `in_deg==True`)
    
    and return quaternion in different formats
    - 'ros' : geometry_msgs/Quaternion
    - 'xyzw': tuple(qx,qy,qz,qw)
    - 'wxyz': tuple(qw,qx,qy,qz)\n

    Debug: return `-1` for invalid format
    """
    if in_deg: roll,pitch,yaw=[math.radians(deg_) for deg_ in (roll,pitch,yaw)]
    qx,qy,qz,qw=quaternion_from_euler(roll,pitch,yaw)
    if qformat=='ros': return Quaternion(x=qx,y=qy,z=qz,w=qw)
    elif qformat=='xyzw': return (qx,qy,qz,qw)
    elif qformat=='wxyz': return (qw,qx,qy,qz)
    else: raise Exception  #TODO: raise proper exception


def adtf_compute_relative_quaternion(q_prev, q_curr, qformat='ros'):
    """
    Compute relative quaternion or difference of two quaternions.\n
    Different quaternion formats are supported:
    - 'ros' : geometry_msgs/Quaternion
    - 'xyzw': tuple(qx,qy,qz,qw)
    - 'wxyz': tuple(qw,qx,qy,qz)\n
    """
    if qformat=='ros': 
        q1=__quaternion_ros_to_tuple__(q_prev)
        q2=__quaternion_ros_to_tuple__(q_curr)
        q1_inv = quaternion_inverse(q1)
        q_rel = quaternion_multiply(q2,q1_inv)
        return __quaternion_tuple_to_ros__(q_rel)
    elif qformat=='xyzw': 
        q_prev_inv = quaternion_inverse(q_prev)
        return quaternion_multiply(q_curr,q_prev_inv)
    elif qformat=='wxyz': 
        q1=(q_prev[3],)+q_prev[0:3]
        q2=(q_curr[3],)+q_curr[0:3]
        q1_inv = quaternion_inverse(q1)
        q_rel = quaternion_multiply(q2,q1_inv)
        return q_rel[1:3]+(q_rel[0],)
    else: raise Exception  #TODO: raise proper exception


def __quaternion_ros_to_tuple__(quat:Quaternion, tuple_order='xyzw') -> tuple:
    if tuple_order=='xyzw': return (quat.x, quat.y, quat.z, quat.w)
    elif tuple_order=='wxyz': return (quat.w, quat.x, quat.y, quat.z)
    else: raise Exception  #TODO: raise proper exception


def __quaternion_tuple_to_ros__(quat:tuple, tuple_order='xyzw') -> Quaternion:
    if tuple_order=='xyzw': return Quaternion(x=quat[0],y=quat[1],z=quat[2],w=quat[3])
    elif tuple_order=='wxyz': return Quaternion(x=quat[1],y=quat[2],z=quat[3],w=quat[0])
    else: raise Exception  #TODO: raise proper exception


def lin_2d_transform_g2b_euler(x,y,yaw):
    x_global = np.array([x,y])
    rot_matr = np.array([
        [ math.cos(yaw), math.sin(yaw)],
        [-math.sin(yaw), math.cos(yaw)]
    ]
    )
    return np.matmul(rot_matr,x_global)


def lin_2d_transform_g2b_quaternion(x,y,yaw):
    x_body = lin_3d_transform_g2b_quaternion(x,y,0,0,0,yaw)
    return x_body[0], x_body[1]


def lin_3d_transform_g2b_quaternion(x,y,z,roll,pitch,yaw):
    q_rot = quaternion_from_euler(-roll,-pitch,-yaw)
    q_rot_inv = quaternion_inverse(q_rot)
    p_global = (x,y,z,0)
    p_body = quaternion_multiply(quaternion_multiply(q_rot,p_global),q_rot_inv)
    return np.array([p_body[0],p_body[1],p_body[2]])


#--TEST (TODO: use pytest and move to test dir)--#

def test_adtf_euler_from_quaternion():
    quat_=Quaternion()
    yawrad=30*(math.pi/180)
    ix,iy,iz=(0.0,0.0,1.0)
    quat_.x=ix*math.sin(yawrad/2)
    quat_.y=iy*math.sin(yawrad/2)
    quat_.z=iz*math.sin(yawrad/2)
    quat_.w=math.cos(yawrad/2)
    try:
        np.testing.assert_almost_equal(
            yawrad,
            adtf_euler_from_quaternion(quat_)[2]
        )
        print("OK")
    except AssertionError as e:
        print(e)


def test_adtf_quaternion_from_euler():
    yawrad=30*(math.pi/180)
    ix,iy,iz=(0.0,0.0,1.0)
    qx=ix*math.sin(yawrad/2)
    qy=iy*math.sin(yawrad/2)
    qz=iz*math.sin(yawrad/2)
    qw=math.cos(yawrad/2)
    try:
        np.testing.assert_array_almost_equal(
            np.array((qx,qy,qz,qw)),
            np.array(adtf_quaternion_from_euler(0.0,0.0,yawrad,qformat='xyzw'))
        )
        np.testing.assert_array_almost_equal(
            np.array((qw,qx,qy,qz)),
            np.array(adtf_quaternion_from_euler(0.0,0.0,yawrad,qformat='wxyz'))
        )
        np.testing.assert_almost_equal(
            np.array(Quaternion(x=qx,y=qy,z=qz,w=qw)),
            np.array(adtf_quaternion_from_euler(0.0,0.0,yawrad))
        )
        print("OK")
    except AssertionError as e:
        print(e)


def test_lin2d_transform_g2b():
    dt = 0.1
    x_world_arr = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
    y_world_arr = [4.0, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7]
    yaw_world_arr = [math.pi/2 for _ in range(len(x_world_arr))]
    x_0, y_0 = lin_2d_transform_g2b_euler(x_world_arr[0],y_world_arr[0],yaw_world_arr[0])
    x_0_q, y_0_q = lin_2d_transform_g2b_quaternion(x_world_arr[0],y_world_arr[0],yaw_world_arr[0])

    try:
        for i in range(len(x_world_arr)):
            x_body, y_body = lin_2d_transform_g2b_euler(x_world_arr[i],y_world_arr[i],yaw_world_arr[i])
            x_body -= x_0
            y_body -= y_0
            x_body_q, y_body_q = lin_2d_transform_g2b_quaternion(x_world_arr[i],y_world_arr[i],yaw_world_arr[i])
            x_body_q -= x_0_q
            y_body_q -= y_0_q
            # print(f"euler: {np.array([x_body,y_body])}")
            # print(f"quat:  {np.array([x_body_q,y_body_q])}")
            np.testing.assert_array_almost_equal(
                np.array([x_body,y_body]),
                np.array([x_body_q,y_body_q])
            )
            time.sleep(dt)
        print("OK")
    except AssertionError as e:
        print(e)


if __name__ == '__main__':
    test_lin2d_transform_g2b()  #compare 2d euler vs quaternion global2body trafo
    test_adtf_euler_from_quaternion()
