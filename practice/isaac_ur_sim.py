#!/usr/bin/env python3
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Felix Exner

# This is an example of how to interface the robot without any additional ROS components. For
# real-life applications, we do recommend to use something like MoveIt!

import time

import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile



TRAJECTORIES = {
    "traj0": [
        {
            "positions": [0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [-0.195016, -1.70093, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
    "traj1": [
        {
            "positions": [-0.195016, -1.70094, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=0, nanosec=0),
        },
        {
            "positions": [0.30493, -0.982258, 0.955637, -1.48215, -1.72737, 0.204445],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
}


class JTCClient(rclpy.node.Node):
    """Small test client for the jtc."""

    def __init__(self):
        super().__init__("jtc_client")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')


        self.subscription = self.create_subscription(
            JointState,
            'ur_joint_states',
            self.joint_state_callback,
            QoSProfile(depth=10)
        )

        self.timer = self.create_timer(5.0, self.timer_callback)


        self.joint_positions = [0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18]
        self.time_stamp = 0.0
        self.goals = None
        self.joint_state_enable = False

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None


    def create_trajectory(self,msg):
        self.time_stamp_sec = msg.header.stamp.sec
        self.time_stamp_nsec = msg.header.stamp.nanosec

        goal = JointTrajectory()
        goal.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #point.effort =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #point.accelerations =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #point.time_from_start = Duration(sec=self.time_stamp_sec, nanosec=self.time_stamp_sec)
        point.time_from_start = Duration(sec=4, nanosec=0)

        goal.points.append(point)
        self.goals = goal
        #self.get_logger().info(f"Received trajectory: {self.goals}")

    def joint_state_callback(self, msg):
        self.joint_positions = msg.position

        #Create trajectory
        self.create_trajectory(msg)

        self.joint_state_enable = True

    def timer_callback(self):
        self.get_logger().info("Timer callback triggered")

        self.execute_trajectory_new()


    def execute_trajectory_new(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals

        #goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        #goal.goal_tolerance = [
        #    JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(6)
        #]

        length_traj = 0

        try:
            length_traj = len(goal.trajectory.points)
        except:
            length_traj = 0

        if(length_traj == 1):
            self._send_goal_future = self._action_client.send_goal_async(goal)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(f"Executing trajectory ")
            print (goal.trajectory.points)

        else:
            print("No trajectory found")


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            #raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(0.01)
        #    self.execute_next_trajectory()
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Done with result: {self.error_code_to_str(result.error_code)}"
                )
            #raise RuntimeError("Executing trajectory failed. " + result.error_string)

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"


def main(args=None):
    rclpy.init(args=args)

    jtc_client = JTCClient()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(jtc_client)

    try:
        executor.spin()
    except RuntimeError as err:
        jtc_client.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("jtc_client").info("Done")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
