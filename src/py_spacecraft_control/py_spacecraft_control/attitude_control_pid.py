import rclpy
import numpy as np
from spacecraft_msgs.msg import StateSpaceVector
from builtin_interfaces.msg import Time
from std_msgs.msg import Float32MultiArray
import transforms3d.euler
from py_spacecraft_control.state_space_publisher import state_vector
from math import radians
from geometry_msgs.msg import Wrench, Vector3

from rclpy.node import Node

Kp = 0.25
Ki = 1
Kd = 0.6

PID_PERIOD = 0.1


def normalize_quat( q):
    return np.array(q) / np.linalg.norm(q)

class sc_attitude_control_pid(Node):

    def __init__(self):
        super().__init__("attitude_control_pid")

        self.state_space_sub = self.create_subscription(StateSpaceVector, 'spacecraft/state_space_vector', self.state_space_callback, 10)
        self.torque_publisher = self.create_publisher(Wrench, '/spacecraft/ThrusterBodyWrench', 10)

        self.pid_time = self.create_timer(PID_PERIOD, self.pid)

        self.stamp = 0.0
        self.state_space_vec = None

    def state_space_callback(self, msg: StateSpaceVector):

        #self.stamp = msg.stamp.second + msg.stamp.nanoseconds/1e9
        self.state_space_vec = state_vector(msg.state_vector.data)

    def pid(self):

        if self.state_space_vec is not None:

            roll = radians(0.0)   # x
            pitch = radians(5.0)  # y
            yaw = radians(65.0)    # z
            des_quat = transforms3d.euler.euler2quat(roll, pitch, yaw, axes='sxyz')
            cur_quat = self.state_space_vec.get_orientation_quat()

            cur_q_t3d = [cur_quat[3], cur_quat[0], cur_quat[1], cur_quat[2]]

            q_error = transforms3d.quaternions.qmult(
                des_quat,
                transforms3d.quaternions.qinverse(cur_q_t3d)
            )

            if q_error[0] < 0:
                q_error = [-x for x in q_error]

            axis, angle = transforms3d.quaternions.quat2axangle(q_error)
            e_rot = np.array(axis) * angle

            cur_ang_vel = np.array(self.state_space_vec.get_angular_vel())

            torque = Kp * e_rot + Kd * (-cur_ang_vel)

            msg = Wrench()

            msg.torque.x = torque[0]
            msg.torque.y = torque[1]
            msg.torque.z = torque[2] 

            self.get_logger().info(str(msg))


            self.torque_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = sc_attitude_control_pid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()