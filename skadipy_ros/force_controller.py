#!/usr/bin/env python3

import rclpy
import rclpy.node

import geometry_msgs.msg

import numpy as np
import skadipy
from skadipy.allocator.reference_filters import MinimumMagnitudeAndAzimuth

class ForceControllerROS(rclpy.node.Node):
    def __init__(self):
        super().__init__("force_controller")


        self.actuators: list = []
        self.allocator: skadipy.allocator.AllocatorBase = None

        self.tau_cmd = np.zeros((6, 1), dtype=np.float32)
        self._initialize_thrusters()
        self._initialize_allocator()

        self.subs = {}
        self.subs["force"] = self.create_subscription(
            geometry_msgs.msg.Wrench, "control/force", self.force_callback, 10
        )

        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):

        self.allocator.allocate(tau=self.tau_cmd)

        self.get_logger().info(f"Allocated {self.allocator.allocated}")

    def force_callback(self, msg: geometry_msgs.msg.Wrench):

        self.tau_cmd = np.array(
            [
                msg.force.x,
                msg.force.y,
                msg.force.z,
                msg.torque.x,
                msg.torque.y,
                msg.torque.z,
            ],
            dtype=np.float32,
        ).reshape((6, 1))

    def _initialize_thrusters(self):
        tunnel = skadipy.actuator.Fixed(
            position=skadipy.toolbox.Point([0.3875, 0.0, 0.0]),
            orientation=skadipy.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), radians=np.pi / 2.0
            ),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 1.0,
                "name": "tunnel",
            },
        )
        port_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4574, -0.055, 0.0]),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 1.0,
                "reference_angle": np.pi / 4.0,
                "name": "port_azimuth",
            },
        )
        starboard_azimuth = skadipy.actuator.Azimuth(
            position=skadipy.toolbox.Point([-0.4547, 0.055, 0.0]),
            extra_attributes={
                "rate_limit": 0.1,
                "saturation_limit": 1.0,
                "reference_angle": -np.pi / 4.0,
                "name": "starboard_azimuth",
            },
        )

        # Put all actuators in a list and create the allocator object
        self.actuators = [
            tunnel,
            port_azimuth,
            starboard_azimuth,
        ]

    def _initialize_allocator(self):
        dofs = [
            skadipy.allocator.ForceTorqueComponent.X,
            skadipy.allocator.ForceTorqueComponent.Y,
            skadipy.allocator.ForceTorqueComponent.N,
        ]
        self.allocator = skadipy.allocator.reference_filters.MinimumMagnitudeAndAzimuth(
            actuators=self.actuators,
            force_torque_components=dofs,
            gamma=0.001,
            mu=0.01,
            rho=10,
            time_step=0.01,
            control_barrier_function=skadipy.safety.ControlBarrierFunctionType.ABSOLUTE,
        )
        self.allocator.compute_configuration_matrix()


def main(args=None):
    rclpy.init(args=args)
    force_controller = ForceControllerROS()
    rclpy.spin(force_controller)
    force_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
