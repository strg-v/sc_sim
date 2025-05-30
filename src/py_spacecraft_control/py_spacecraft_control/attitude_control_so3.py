import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
import rclpy.time
import rclpy.time_source
from spacecraft_msgs.msg import StateSpaceVector
from py_spacecraft_control.state_space_publisher import state_vector
import numpy as np
import transforms3d as tf

CONTROL_LOOP_PERIOD = 0.15
Kr = 0.1
Kw = 0.95 * Kr

SLERP_STEPS = 0

class sc_attitude_control_so3(Node):

    remaining_slerp = 0
    q_des = None

    def __init__(self):
        super().__init__("attutude_control_so3")

        self.attitute_sub = self.create_subscription(Vector3, 'spacecraft/attitute_euler', self.attitute_euler_callback, 10)
        self.state_space_sub = self.create_subscription(StateSpaceVector, 'spacecraft/state_space_vector', self.state_space_callback, 10)
        self.torque_publisher = self.create_publisher(Wrench, '/spacecraft/ThrusterBodyWrench', 10)

        self.control_loop_timer = self.create_timer(CONTROL_LOOP_PERIOD, self.control_loop)

        self.stamp = 0.0
        self.state_space_vec = None
        self.max_slerp_time = 5

        cube_length = 0.3       # m
        mass = 3                # kg
        self.I = 1/6 * mass * cube_length**2 * np.eye(3)

        self.w_des = np.array([
            [0.0],
            [0.0],
            [0.0],
        ])

    def state_space_callback(self, msg: StateSpaceVector):

        #self.stamp = msg.stamp.second + msg.stamp.nanoseconds/1e9
        self.state_space_vec = state_vector(msg.state_vector.data)

    def attitute_euler_callback(self, msg: Vector3):

        self.des_attitute_euler = np.array([
            [msg.x],
            [msg.y],
            [msg.z],
        ])

        roll = np.deg2rad(self.des_attitute_euler[0])      # x
        pitch = np.deg2rad(self.des_attitute_euler[1])     # y
        yaw = np.deg2rad(self.des_attitute_euler[2])       # z

        q_des = tf.euler.euler2quat(roll, pitch, yaw)

        if q_des[3] < 0:
            q_des = -q_des

        self.q_des = q_des
    

    def control_loop(self):

        # check if current orientation is available
        if self.state_space_vec is None:
            return
        
        # check if desiered orientation is available
        if self.q_des is None:
            return
        
        # Get current rotational matrix from quaternion
        quat_init = self.state_space_vec.get_orientation_quat() # x, y, z, w

        if quat_init[3] < 0:
            quat_init = -quat_init

        quat_init_wxyz = np.array([
            quat_init[3],
            quat_init[0],
            quat_init[1],
            quat_init[2],
        ])

        R_d = tf.quaternions.quat2mat(self.q_des)

        #R_curr = np.array(q_mat_curr[0:3, 0:3])
        R_curr = tf.quaternions.quat2mat(quat_init_wxyz)

        R_e = R_d.T @ R_curr

        e_R_mat = 0.5 * (R_e - R_e.T)

        e_R_vec = np.array([
            [e_R_mat[2][1]],
            [e_R_mat[0][2]],
            [e_R_mat[1][0]],
        ])

        # Get current angular velocities
        w_cur = self.state_space_vec.get_angular_vel()
        
        e_w = w_cur - (R_curr.T @ (R_d @ self.w_des))

        self.get_logger().info(str(e_w))


        Iw = self.I @ w_cur
        wxIw = np.cross(w_cur.T, Iw.T).T * 1e15

        tau = -Kr*e_R_vec -Kw*e_w #+ wxIw


        tau = np.clip(tau, -0.05, 0.05)

        for m in tau:
            if np.isnan(m):
                return

        #self.get_logger().info(str(tau))

        msg = Wrench()

        msg.torque.x = tau[0][0]
        msg.torque.y = tau[1][0]
        msg.torque.z = tau[2][0] 
        self.torque_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = sc_attitude_control_so3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


