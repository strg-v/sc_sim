from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Wrench, Vector3
from spacecraft_msgs.msg import ThrustCommand
import yaml
import numpy as np
from scipy.optimize import lsq_linear
from ament_index_python.packages import get_package_share_directory
import os
from builtin_interfaces.msg import Duration
from copy import copy
import cvxpy as cp

# PWM Period
T = 0.1

class thruster:

    def __init__(self, force_vector, r, name: str, parent_node: Node):
        # force_vector: direction and force of thruster
        # r: position of thruster in reference to body frame
        # name: name of thruster, used to create publisher 
        # parent_node: used to create publisher
        self.r = np.array(r)
        self.force_vector = np.array(force_vector)

        self.parent_node = parent_node
        self.name = name
        self.publisher = parent_node.create_publisher(ThrustCommand, f'/spacecraft/thruster/{name}', 10)

    def fire(self, duration: float):
        # Fires the thruster with set direction and given duration
        cmd = ThrustCommand()
        cmd.direction_selection = Vector3(
            x=float(self.force_vector[0]),
            y=float(self.force_vector[1]),
            z=float(self.force_vector[2]),
        )

        dur = Duration()
        # Clip thruster duration between PWM period and 0
        duration = np.clip(duration, 0.0, T)
        dur.nanosec = int(duration * 1e9)
        cmd.duration = dur

        # Send message to fire thruster
        self.publisher.publish(cmd)


class thrusterAssembly:

    def __init__(self, thrusters = []):

        self.thrusters = []
        self.allocation_matrix = np.array([])

        # Add given thruster list and calculate allocation matrix
        if len(thrusters) > 0:
            for thr in thrusters:
                self.add_thruster(thr)

    def add_thruster(self, thr: thruster):
        # Adds thruster to the assembly and calculates allocation matrix
        self.thrusters.append(thr)
        self.calculate_alloc_mat()

    def calculate_alloc_mat(self):
        # calculates the thruster allocation matrix for this group
        allocation_matrix = list()

        for thr in self.thrusters:                

            r = thr.r
            d = thr.force_vector

            torque = np.cross(r, d)
            b = np.concatenate([d, torque])
            allocation_matrix.append(b)
        
        allocation_matrix = np.array(allocation_matrix)

        self.allocation_matrix = allocation_matrix

    def fire(self, durations: list):
        # Fires every thruster in this thruster assembly group
        if len(durations) != 4:
            self.parent_node.get_logger().warn(f"[{self.name}] Number of durations does not match number of thrusters!")
            return
        
        for i, dur in enumerate(durations):

            self.thrusters[i].fire(dur)
        

class thruster_mapper_node(Node):

    apply_durations = None

    def __init__(self):
        super().__init__("thruster_mapper")

        force = 0.5

        # Define all 4 thrusters positioned in x+ directon
        thr_xp_pos = [0.175, 0, 0]
        thr_xp_yp = thruster(force * np.array([0, 1, 0]), thr_xp_pos, "xp_yp", self)
        thr_xp_ym = thruster(force * np.array([0, -1, 0]), thr_xp_pos, "xp_ym", self)
        thr_xp_zp = thruster(force * np.array([0, 0, 1]), thr_xp_pos, "xp_zp", self)
        thr_xp_zm = thruster(force * np.array([0, 0, -1]), thr_xp_pos, "xp_zm", self)
        self.assembly_xp = thrusterAssembly([thr_xp_yp, thr_xp_ym, thr_xp_zp, thr_xp_zm])

        # Define all 4 thrusters positioned in x- directon
        thr_xm_pos = [-0.175, 0, 0]
        thr_xm_yp = thruster(force * np.array([0, 1, 0]), thr_xm_pos, "xm_yp", self)
        thr_xm_ym = thruster(force * np.array([0, -1, 0]), thr_xm_pos, "xm_ym", self)
        thr_xm_zp = thruster(force * np.array([0, 0, 1]), thr_xm_pos, "xm_zp", self)
        thr_xm_zm = thruster(force * np.array([0, 0, -1]), thr_xm_pos, "xm_zm", self)
        self.assembly_xm = thrusterAssembly([thr_xm_yp, thr_xm_ym, thr_xm_zp, thr_xm_zm])

        # Define all 4 thrusters positioned in y+ directon
        thr_yp_pos = [0, 0.175, 0]
        thr_yp_xp = thruster(force * np.array([1, 0, 0]), thr_yp_pos, "yp_xp", self)
        thr_yp_xm = thruster(force * np.array([-1, 0, 0]), thr_yp_pos, "yp_xm", self)
        thr_yp_zp = thruster(force * np.array([0, 0, 1]), thr_yp_pos, "yp_zp", self)
        thr_yp_zm = thruster(force * np.array([0, 0, -1]), thr_yp_pos, "yp_zm", self)
        self.assembly_yp = thrusterAssembly([thr_yp_xp, thr_yp_xm, thr_yp_zp, thr_yp_zm])

        # Define all 4 thrusters positioned in y- directon
        thr_ym_pos = [0, -0.175, 0]
        thr_ym_xp = thruster(force * np.array([1, 0, 0]), thr_ym_pos, "ym_xp", self)
        thr_ym_xm = thruster(force * np.array([-1, 0, 0]), thr_ym_pos, "ym_xm", self)
        thr_ym_zp = thruster(force * np.array([0, 0, 1]), thr_ym_pos, "ym_zp", self)
        thr_ym_zm = thruster(force * np.array([0, 0, -1]), thr_ym_pos, "ym_zm", self)
        self.assembly_ym = thrusterAssembly([thr_ym_xp, thr_ym_xm, thr_ym_zp, thr_ym_zm])

        # Combine all thruster allocation matricies to one big matrix
        self.thruster_alloc_mat = np.concatenate([
            self.assembly_xp.allocation_matrix,
            self.assembly_xm.allocation_matrix,
            self.assembly_yp.allocation_matrix,
            self.assembly_ym.allocation_matrix
        ])

        self.get_logger().info("\n" + str(self.thruster_alloc_mat))
        
        # Number of thrusters for solver variables
        self.thruster_count = len(self.thruster_alloc_mat)

        # Subscriber to wrench messages
        self.wrench_sub = self.create_subscription(Wrench, '/spacecraft/ThrusterBodyWrench', self.wrench_callback, 10)

        # PWM timer to fire thrusters periodically
        self.time = self.create_timer(T, self.pwm_callback)
        
    def pwm_callback(self):
        
        # Apply thruster force every pwm period
        if self.apply_durations is not None and len(self.apply_durations) == self.thruster_count:
            
            self.assembly_xp.fire(self.apply_durations[0:4])
            self.assembly_xm.fire(self.apply_durations[4:8])
            self.assembly_yp.fire(self.apply_durations[8:12])
            self.assembly_ym.fire(self.apply_durations[12:16])

    def wrench_callback(self, wrench: Wrench):

        # desired force and torque from message
        self.desired = np.array([
            wrench.force.x, wrench.force.y, wrench.force.z,
            wrench.torque.x, wrench.torque.y, wrench.torque.z
        ])

        # t will hold the on time for each thruster
        t = cp.Variable(self.thruster_count)
        # alpha [0..1] will scale down incoming message so that solver
        # can achieve solution in case request is physical not feasible
        alpha = cp.Variable()
        # allows small deviations from actual desired value
        # makes this implementation more reliable
        # penaltiy is applied later when slack is required
        slack_output = cp.Variable(6)
        
        max_slack = T/4.0
        B = self.thruster_alloc_mat.T

        # Constraints for the solver
        constraints = [
            # minimum duation for thruster is 0, time must not be below 0
            t >= 0.0,
            # myximum duration is equal to full PWM period
            t <= T,
            # Constraints the scaling factor between 0% and 100%
            alpha >= 0.0,
            alpha <= 1.0,
            # Equation to solve:
            # T * self.desired is the impulse we are looking for
            # alpha scales the desiered impulse down in case it is not feasable with thruster constraints
            # (@) is matrix dot multiplication
            # B @ t is the resulting impulse (aim: T * self.desired)
            # slack_output is the deviation from the goal
            B @ t + slack_output == T * alpha * self.desired,
            # Limits for the slack
            slack_output >= -max_slack,
            slack_output <= max_slack,
        ]

        # Objective for the solver
        # Minimize the equation
        objective = cp.Minimize(
            # Square sum of duration to get most efficient firing
            cp.sum_squares(t) +
            # Penalty of square sum of slack
            10_000 * cp.sum_squares(slack_output) -
            # Penalty of alpha getting low
            10_000 * alpha
        )

        # Specify porblem with constraints and solve
        prob = cp.Problem(objective, constraints)
        result = prob.solve()

        # Check if valid result exists
        if not np.isnan(result):
            self.get_logger().info("Durations: " + str(t.value) +
                                    "\nResult: " + str(B @ t.value / T) +
                                    "\nAlpha: " + str(alpha.value) +
                                    "\nSlack: " + str(slack_output.value)
            )
            # Mark result as available
            self.apply_durations = t.value
            self.fire = True  
        else:
            self.get_logger().warn("No solution for wrench found.")

    
def main(args=None):
    rclpy.init(args=args)
    node = thruster_mapper_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()