import rclpy
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from std_msgs.msg import Float32MultiArray
import numpy as np
from spacecraft_msgs.msg import StateSpaceVector

from rclpy.node import Node

publish_period = 0.05

class state_vector:

    def __init__(self, state_vector_arr = []):

        self.state_vector = state_vector_arr

    def get_orientation_quat(self) -> list:
    
        if len(self.state_vector) != 13:
            raise Exception

        return np.array([
            self.state_vector[3],
            self.state_vector[4],
            self.state_vector[5],
            self.state_vector[6],
        ])

    def get_angular_vel(self):
        if len(self.state_vector) != 13:
            raise Exception

        return np.array([
            [self.state_vector[10]],
            [self.state_vector[11]],
            [self.state_vector[12]],
        ])

class sc_state_space_publisher(Node):

    def __init__(self):
        super().__init__("sc_state_space_publisher")

        self.state_vector = np.zeros([13,1])
        self.stamp = Time(sec=0, nanosec=0)
        self.valid = False

        self.gazebo_state_sub = self.create_subscription(Odometry, '/spacecraft/state', self.gazebo_state_callback, 10)        
        self.state_space_vec_pub = self.create_publisher(StateSpaceVector, 'spacecraft/state_space_vector', 10)

        self.publish_timer = self.create_timer(publish_period, self.publish_state)

    def gazebo_state_callback(self, msg: Odometry):

        self.stamp = msg.header.stamp

        self.state_vector = Float32MultiArray(data=[
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])

        self.valid = True

    def publish_state(self):

        if self.valid:
            state_msg = StateSpaceVector()
            state_msg.stamp = self.stamp
            state_msg.state_vector = self.state_vector

            self.state_space_vec_pub.publish(state_msg)

            self.valid = False

def main(args=None):
    rclpy.init(args=args)
    node = sc_state_space_publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()