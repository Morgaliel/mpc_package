###########################################################################################
#                                                                                         #
#                                                                                         #
#     Files placed in f1tenth_mpc_lib (except pid.py) directory are based on:             #
#     * Github repository: Multi-Purpose MPC                                              #
#     * Author: matssteinweg, ZTH                                                         #
#     All rights reserved to the original author.                                         #
#     For more details please check https://github.com/matssteinweg/Multi-Purpose-MPC     #
#                                                                                         #
#     This is an implementation for AWSIM f1tenth simulator.                              #
#                                                                                         #
#                                                                                         #
###########################################################################################

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from autoware_auto_control_msgs.msg import AckermannControlCommand
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from telemetry_interface.msg import Telemetry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import math
from scipy import sparse

from mpc_package.f1tenth_mpc_lib.reference_path import ReferencePath
from mpc_package.f1tenth_mpc_lib.map import Map, Obstacle
from mpc_package.f1tenth_mpc_lib.spatial_bicycle_models import BicycleModel
from mpc_package.f1tenth_mpc_lib.MPC import MPC
import mpc_package.f1tenth_mpc_lib.pid as pid

ENABLE_CONTROL = True

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q 

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class f1tenth_mpc_node(Node):
    def __init__(self):
        super().__init__('f1tenth_mpc_node')
        
        self.imu_sub = self.create_subscription(Imu,
                                                '/sensing/vesc/imu',
                                                self.imu_callback,
                                                10)
        
        self.ground_truth_sub = self.create_subscription(PoseStamped,   
                                                        '/ground_truth/pose',
                                                        self.ground_truth_callback,
                                                        10)

        group_name = '/MPC'                    

        qos_profile = QoSProfile(depth=50,  
                                 durability=QoSDurabilityPolicy.
                                 RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.control_pub = self.create_publisher(AckermannControlCommand,
                                                '/control/command/control_cmd',
                                                qos_profile)

        self.telemetry_pub = self.create_publisher(Telemetry,
                                                group_name + '/telemetry',
                                                10)

        self.path_pub = self.create_publisher(PoseArray,
                                              group_name + '/path',
                                              10)
        
        self.path_pub_mpc = self.create_publisher(PoseArray,
                                              group_name + '/path_mpc',
                                              10)

        self.timer = self.create_timer(1, self.path_publish_callback)
        
        '''
        Setup reference path
        '''

        map_file = '/home/jan/autoware/src/mpc_package/maps/imola_flipped.png'

        map1= Map(file_path=map_file, origin=[-30.5, 48.05], resolution=0.05)
        
        wp_x = [-1.2,   0.0,    3.0,    6.8,    10.6,   14.3,   18.0,   21.6,   25.2,   28.4,
                29.5,   30.0,   32.8,   34.9,   35.8,   36.5,   37.1,   37.4,   37.1,   36.1,   
                38.4,   41.1,   42.9,   42.5,   40.2,   36.7,   33.2,   29.7,   26.0,   22.3,   
                18.6,   16.8,   15.7,   15.1,   15.9,   17.3,   18.9,   18.4,   17.1,   11.2,   
                7.5,    3.9,    0.2,    -3.5,   -7.1,   -10.5,  -13.4,  -16.0,  -18.3,  -20.5,  
                -22.9,  -25.8,  -28.9,  -28.9, -27.7,  -26.0,   -24.3,  -21.0,  -17.9,  -14.8,  
                -11.2,  -7.6,   -5.3,   -4.5,   -3.5,   -0.7,   1.2]

        wp_y = [0.0,    0.0,    0.3,    0.2,    0.2,    0.5,    1.2,    2.2,    3.3,    4.6,    
                6.2,    8.4,    11.2,   14.2,   17.9,   21.6,   25.2,   29.0,   31.7,   35.4,   
                38.7,   41.2,   44.5,   46.2,   46.5,   45.2,   43.9,   42.6,   41.8,   41.7,   
                41.1,   40.2,   38.8,   35.1,   31.5,   28.1,   24.5,   20.8,   19.1,   18.3,   
                17.7,   17.0,   16.1,   15.5,   14.6,   13.0,   10.7,   7.9,    5.0,    1.9,    
                -0.9,   -3.4,   -5.8,   -7.45,  -9.5,   -10.5,  -10.1,  -8.1,   -6.1,   -4.2,   
                -3.0,   -2.6,   -1.7,   -0.8,   -0.1,   0.0,    0.1]


        path_resolution = 0.5  # m / wp
        self.t = 0

        reference_path = ReferencePath(
            map1,
            wp_x,
            wp_y,
            path_resolution,
            smoothing_distance=2, # (baseline = 5)
            max_width=1,
            circular=True,
        )

        '''
        Setup model
        '''
        self.Vehicle = BicycleModel(length=0.324, width=0.3,
                           reference_path=reference_path, Ts=0.04)

        N = 20 #horizon
        Q = sparse.diags([0.5, 0.0, 0.0])   #state cost matrix
        R = sparse.diags([0.1, 0.0])        #input cost matrix
        QN = sparse.diags([0.0, 0.0, 0.0])  #final state cost matrix

        v_max = 7.0  # m/s baseline = 5
        self.delta_max = np.pi/3  # rad
        ay_max = 8.0  # m/s^2 baseline = 8
        self.ax_min = -1 # m/s^2 (baseline = -1)
        self.ax_max = 1 # m/s^2 (baseline = 1)

        InputConstraints = {'umin': np.array([0.0, -np.tan(self.delta_max)/self.Vehicle.length]),
                            'umax': np.array([v_max, np.tan(self.delta_max)/self.Vehicle.length])}
        StateConstraints = {'xmin': np.array([-np.inf, -np.inf, -np.inf]),
                            'xmax': np.array([np.inf, np.inf, np.inf])}
        self.Controller = MPC(self.Vehicle, N, Q, R, QN, StateConstraints, InputConstraints, ay_max)



        SpeedProfileConstraints = {
            'a_min': self.ax_min,
            'a_max': self.ax_max,
            'v_min': 0.0,
            'v_max': v_max,
            'ay_max': ay_max,
        }
        self.Vehicle.reference_path.compute_speed_profile(SpeedProfileConstraints)

        '''
        Set initial state
        '''
        self.t = 0
        self.delta = 0.0
        self.delta_curve  = 0.0
        self.ref_speed_path = 0.0
        self.last_pose = np.array([0, 0])
        self.speed = 0.2
        self.acc_command = 0.0
        self.yaw_rate = 0.0
        self.get_logger().info('MPC node is started')

        ## Longitudinal Controller
        self.pid = pid.PID(Kp=8.0, Ki=0.05, Kd=0.04, dt=0.02)

    def path_publish_callback(self):
        # publish waypoints alongside the reference path
        path_msg = PoseArray()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for i in range(len(self.Vehicle.reference_path.waypoints)):
            pose = Pose()
            pose.position.x = self.Vehicle.reference_path.waypoints[i].x
            pose.position.y = self.Vehicle.reference_path.waypoints[i].y
            pose.orientation = quaternion_from_euler(0, 0, self.Vehicle.reference_path.waypoints[i].psi)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def publish_pred_path(self, x, y, yaw):
        # publish path given by the MPC
        path_msg = PoseArray()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for i in range(len(x)):
            pose = Pose()
            pose.position.x = float(x[i])
            pose.position.y = float(y[i])
            pose.orientation = quaternion_from_euler(0, 0, float(yaw[i]))
            path_msg.poses.append(pose)
        self.path_pub_mpc.publish(path_msg)

    def imu_callback(self, msg):
        # send controll comands and proceed with speed deduction
        self.imu_msg = msg
        self.yaw_rate = msg.angular_velocity.z    
        delta_to_send = np.clip(self.delta, -self.delta_max, self.delta_max)    
        if(delta_to_send >0.15):
            delta_to_send = delta_to_send * 2.0
        elif(delta_to_send < -0.15):
            delta_to_send = delta_to_send * 2.0
        control_cmd = AckermannControlCommand()
        control_cmd.stamp = self.get_clock().now().to_msg()
        control_cmd.longitudinal.acceleration = self.acc_command
        control_cmd.lateral.steering_tire_angle = delta_to_send
        if(ENABLE_CONTROL):
            # self.get_logger().info('control command acc: ' + str(self.acc_command))
            # self.get_logger().info('control command delta: ' + str(delta_to_send))
            self.get_logger().info('v: ' + str(self.speed))
            self.control_pub.publish(control_cmd)

    def ground_truth_callback(self, msg):
        self.ground_truth_msg = msg
        x = self.ground_truth_msg.pose.position.x
        y = self.ground_truth_msg.pose.position.y
        yaw = euler_from_quaternion(self.ground_truth_msg.pose.orientation)[2]
        pose = np.array([x, y, yaw])

        self.speed_x = (x - self.last_pose[0]) / (1 / 25)
        self.speed_y = (y - self.last_pose[1]) / (1 / 25)
        self.speed = np.hypot(self.speed_x, self.speed_y)
        self.last_pose = np.array([x, y])

        self.Vehicle.get_current_waypoint(pose)
        u = self.Controller.get_control() # u[0] - target speed [m/s], u[1] - target steering angle [rad]

        error_pid = u[0] - self.speed
        self.acc_command = np.clip(self.pid.update(error_pid), self.ax_min, self.ax_max) # was 0, 0.3
        self.delta = float(u[1])
 
        # Compute temporal state derivatives
        x_dot = self.speed * np.cos(yaw)
        y_dot = self.speed * np.sin(yaw)
        psi_dot = self.speed / self.Vehicle.length * np.tan(self.delta)
        temporal_derivatives = np.array([x_dot, y_dot, psi_dot])

        # Update spatial state (Forward Euler Approximation)
        self.Vehicle.temporal_state += temporal_derivatives * self.Vehicle.Ts

        # Compute velocity along path
        s_dot = 1 / (1 - self.Vehicle.spatial_state.e_y * self.Vehicle.current_waypoint.kappa) \
                * self.speed * np.cos(self.Vehicle.spatial_state.e_psi)
        
        # Update distance travelled along reference path
        self.Vehicle.s += s_dot * self.Vehicle.Ts

        pred_x = self.Controller.current_prediction[0]
        pred_y = self.Controller.current_prediction[1]
        pred_psi = self.Controller.current_prediction[2]

        self.publish_pred_path(pred_x, pred_y, pred_psi)

        telemetry = Telemetry()
        telemetry.v_x = self.speed_x
        telemetry.v_y = self.speed_y
        telemetry.control_delta = self.delta 
        telemetry.control_acceleration = self.acc_command
        telemetry.v_ref_path = self.Vehicle.current_waypoint.v_ref
        telemetry.v_heading = self.speed
        telemetry.yaw = yaw
        telemetry.distance_on_track = self.Vehicle.s
        self.telemetry_pub.publish(telemetry)

def main(args=None):
    rclpy.init(args=args)
    node = f1tenth_mpc_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()     
