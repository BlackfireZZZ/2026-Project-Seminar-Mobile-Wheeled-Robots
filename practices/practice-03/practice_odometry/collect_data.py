import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion


class DataCollector(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter("duration", 30.0)
        self.declare_parameter("world", "noise_0_1.wbt")
        self.duration = self.get_parameter("duration").value
        self.output_dir = "results"
        self.name = os.path.splitext(self.get_parameter("world").value)[0]

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.clock_listener = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            10
        )
        self.command_listener = self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.command_callback,
            10
        )
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.sim_time = None
        self.start_time = None
        self.last_time = None
        self.commands = []
        self.command_index = 0
        self.command_time = None
        self.v = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.rows = []

    def clock_callback(self, msg: Clock):
        self.sim_time = Time.from_msg(msg.clock).nanoseconds

    def command_callback(self, msg: TwistStamped):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        self.commands.append((stamp, msg.twist.linear.x, msg.twist.angular.z))

    def get_pose(self, transform):
        position = transform.transform.translation
        rotation = transform.transform.rotation
        theta = euler_from_quaternion([
            rotation.x, rotation.y, rotation.z, rotation.w
        ])[2]
        return position.x, position.y, theta

    def integrate(self, stamp):
        dt = (stamp - self.command_time) / 1000000000
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        self.command_time = stamp

    def timer_callback(self):
        if self.sim_time is None or not self.commands:
            return

        try:
            gt = self.tf_buffer.lookup_transform("map", "base_link_gt", Time())
            odom = self.tf_buffer.lookup_transform("map", "base_link", Time())
            stamp = min(
                Time.from_msg(gt.header.stamp).nanoseconds,
                Time.from_msg(odom.header.stamp).nanoseconds,
                self.sim_time
            )
            if self.start_time is not None:
                stamp = min(stamp, self.start_time + round(self.duration * 1000000000))
            if self.last_time is not None and stamp <= self.last_time:
                return
            if stamp < self.commands[0][0]:
                return
            time = Time(nanoseconds=stamp)
            gt = self.tf_buffer.lookup_transform("map", "base_link_gt", time)
            odom = self.tf_buffer.lookup_transform("map", "base_link", time)
        except TransformException:
            return

        gt_x, gt_y, gt_theta = self.get_pose(gt)
        odom_x, odom_y, odom_theta = self.get_pose(odom)

        if self.start_time is None:
            self.start_time = stamp
            self.command_time = stamp
            self.x = gt_x
            self.y = gt_y
            self.theta = gt_theta
            while self.command_index < len(self.commands):
                command_time, v, w = self.commands[self.command_index]
                if command_time > stamp:
                    break
                self.v = v
                self.w = w
                self.command_index += 1
            if self.v == 0 and self.w == 0:
                self.start_time = None
                return

        while self.command_index < len(self.commands):
            command_time, v, w = self.commands[self.command_index]
            if command_time > stamp:
                break
            if command_time > self.command_time:
                self.integrate(command_time)
            self.v = v
            self.w = w
            self.command_index += 1
        self.integrate(stamp)

        elapsed = (stamp - self.start_time) / 1000000000
        odom_position_error = math.hypot(odom_x - gt_x, odom_y - gt_y)
        command_position_error = math.hypot(self.x - gt_x, self.y - gt_y)
        odom_angle_error = abs(math.atan2(
            math.sin(odom_theta - gt_theta), math.cos(odom_theta - gt_theta)
        ))
        command_angle_error = abs(math.atan2(
            math.sin(self.theta - gt_theta), math.cos(self.theta - gt_theta)
        ))
        self.rows.append([
            elapsed, stamp / 1000000000,
            gt_x, gt_y, gt_theta,
            odom_x, odom_y, odom_theta,
            self.x, self.y, self.theta,
            odom_position_error, command_position_error,
            odom_angle_error, command_angle_error
        ])
        self.last_time = stamp

        if stamp >= self.start_time + round(self.duration * 1000000000):
            self.timer.cancel()
            self.save_results()

    def save_results(self):
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, self.name)
        with open(path + ".csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                "time", "sim_time", "gt_x", "gt_y", "gt_yaw",
                "odom_x", "odom_y", "odom_yaw",
                "cmd_x", "cmd_y", "cmd_yaw",
                "odom_position_error", "cmd_position_error",
                "odom_yaw_error", "cmd_yaw_error"
            ])
            writer.writerows(self.rows)
        data = np.array(self.rows)
        figure, axes = plt.subplots(1, 3, figsize=(16, 5))
        axes[0].plot(data[:, 2], data[:, 3], color="black", label="Истинная")
        axes[0].plot(data[:, 5], data[:, 6], color="blue", label="Энкодеры")
        axes[0].plot(data[:, 8], data[:, 9], color="orange", label="Команды")
        axes[0].set_xlabel("X, м")
        axes[0].set_ylabel("Y, м")
        axes[0].set_aspect("equal")
        axes[1].plot(data[:, 0], data[:, 11], color="blue", label="Энкодеры")
        axes[1].plot(data[:, 0], data[:, 12], color="orange", label="Команды")
        axes[1].set_xlabel("Время, с")
        axes[1].set_ylabel("Ошибка положения, м")
        axes[2].plot(data[:, 0], data[:, 13], color="blue", label="Энкодеры")
        axes[2].plot(data[:, 0], data[:, 14], color="orange", label="Команды")
        axes[2].set_xlabel("Время, с")
        axes[2].set_ylabel("Ошибка угла, рад")
        for axis in axes:
            axis.legend()
        figure.savefig(path + ".png")
        plt.close(figure)


def main(args=None):
    rclpy.init(args=args)
    collector = DataCollector("data_collector")
    rclpy.spin(collector)
    collector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
