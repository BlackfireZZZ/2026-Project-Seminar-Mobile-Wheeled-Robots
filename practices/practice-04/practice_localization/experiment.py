import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from practice_localization.metrics import Convergence, pose_error


def pose(transform):
    p = transform.transform.translation
    q = transform.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw


class Experiment(Node):
    def __init__(self):
        super().__init__('experiment')
        defaults = {
            'duration': 180.0,
            'position_threshold': 0.2,
            'angle_threshold': 0.2,
            'confirmation_time': 10.0,
            'localization_mode': 'initial',
            'error_x': 0.15,
            'error_y': -0.15,
            'error_yaw': 0.2,
            'min_particles': 300,
            'max_particles': 1000,
            'experiment_name': 'initial_small_low',
            'output_dir': 'results',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        for name in ('duration', 'position_threshold', 'angle_threshold', 'confirmation_time'):
            if self.settings[name] <= 0:
                raise ValueError(f'{name} must be positive')
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.tf, self)
        self.velocity = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose, 10)
        self.state_client = self.create_client(GetState, '/amcl/get_state')
        self.global_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.state_call = None
        self.global_call = None
        self.initialized = False
        self.pose_received = False
        self.start = None
        self.last_stamp = None
        self.rows = []
        self.done = False
        self.failed = False
        self.wait_start = None
        self.convergence = Convergence(
            self.settings['position_threshold'], self.settings['angle_threshold'],
            self.settings['confirmation_time'],
        )
        self.timer = self.create_timer(0.1, self.tick)

    def amcl_pose(self, message):
        self.pose_received = True

    def initialize(self):
        if self.global_call is not None:
            if self.global_call.done():
                self.global_call.result()
                self.initialized = True
            return
        if not self.state_client.service_is_ready():
            return
        if self.state_call is None:
            self.state_call = self.state_client.call_async(GetState.Request())
            return
        if not self.state_call.done():
            return
        active = self.state_call.result().current_state.id == 3
        self.state_call = None
        if not active:
            return
        if self.settings['localization_mode'] == 'global':
            if self.global_client.service_is_ready():
                self.global_call = self.global_client.call_async(Empty.Request())
            return
        if self.initial_pose.get_subscription_count() == 0:
            return
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = 2.5 + self.settings['error_x']
        message.pose.pose.position.y = self.settings['error_y']
        angle = math.pi / 2 + self.settings['error_yaw']
        message.pose.pose.orientation.z = math.sin(angle / 2)
        message.pose.pose.orientation.w = math.cos(angle / 2)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = (math.pi / 12) ** 2
        self.initial_pose.publish(message)
        self.initialized = True

    def drive(self, moving):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        if moving:
            command.twist.linear.x = 0.2
            command.twist.angular.z = 0.08
        self.velocity.publish(command)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0 or self.done:
            return
        if self.wait_start is None:
            self.wait_start = now
        if self.start is None and now - self.wait_start > 120:
            self.drive(False)
            self.failed = True
            self.done = True
            self.get_logger().error('AMCL initialization timed out')
            return
        try:
            gt = self.tf.lookup_transform('map', 'base_link_gt', Time())
            self.tf.lookup_transform('odom', 'base_link', Time())
        except TransformException:
            return
        if not self.initialized:
            self.drive(False)
            self.initialize()
        elif self.pose_received:
            try:
                estimate = self.tf.lookup_transform('map', 'base_link', Time())
                stamp = min(Time.from_msg(gt.header.stamp).nanoseconds,
                            Time.from_msg(estimate.header.stamp).nanoseconds)
                gt = self.tf.lookup_transform('map', 'base_link_gt', Time(nanoseconds=stamp))
                estimate = self.tf.lookup_transform('map', 'base_link', Time(nanoseconds=stamp))
            except TransformException:
                self.convergence.reset()
                return
            if self.start is None:
                self.start = now
                self.get_logger().info('Initialization complete, collecting data')
            self.drive(True)
            if self.last_stamp is None or stamp > self.last_stamp:
                if stamp * 1e-9 >= self.start:
                    if self.last_stamp is not None and (stamp - self.last_stamp) * 1e-9 > 0.5:
                        self.convergence.reset()
                    t = stamp * 1e-9 - self.start
                    real, guess = pose(gt), pose(estimate)
                    distance, angle = pose_error(real, guess)
                    self.convergence.update(t, distance, angle)
                    self.rows.append((t, *real, *guess, distance, angle))
                self.last_stamp = stamp
            if now - self.start >= self.settings['duration']:
                self.drive(False)
                self.save()
                self.done = True

    def save(self):
        if not self.rows:
            raise RuntimeError('No synchronized TF samples collected')
        folder = Path(self.settings['output_dir'])
        folder.mkdir(parents=True, exist_ok=True)
        base = folder / self.settings['experiment_name']
        with base.with_suffix('.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time', 'gt_x', 'gt_y', 'gt_yaw', 'amcl_x', 'amcl_y', 'amcl_yaw', 'position_error', 'angle_error'])
            writer.writerows(self.rows)
        result = dict(self.settings)
        result.update(t0=self.start, convergence_time=self.convergence.result, samples=len(self.rows))
        base.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
        data = np.array(self.rows)
        fig, axes = plt.subplots(3, 1, figsize=(10, 13), constrained_layout=True)
        titles = {
            'initial_small_low': 'Малая начальная ошибка, мало частиц',
            'initial_small_high': 'Малая начальная ошибка, много частиц',
            'initial_large_low': 'Большая начальная ошибка, мало частиц',
            'initial_large_high': 'Большая начальная ошибка, много частиц',
            'global_low': 'Глобальная локализация, мало частиц',
            'global_high': 'Глобальная локализация, много частиц',
        }
        fig.suptitle(titles[self.settings['experiment_name']])
        axes[0].plot(data[:, 1], data[:, 2], label='Реальный путь')
        axes[0].plot(data[:, 4], data[:, 5], label='Оценка AMCL')
        axes[0].set(xlabel='X, м', ylabel='Y, м', aspect='equal')
        for ax, column, threshold, label in [
            (axes[1], 7, self.settings['position_threshold'], 'Ошибка положения, м'),
            (axes[2], 8, self.settings['angle_threshold'], 'Ошибка угла, рад'),
        ]:
            ax.plot(data[:, 0], data[:, column], label='Ошибка')
            ax.axhline(threshold, color='red', linestyle='--', label='Порог')
            if self.convergence.result is not None:
                ax.axvline(self.convergence.result, color='green', linestyle='--', label='Сошлось')
            ax.set_ylabel(label)
        axes[2].set_xlabel('Время, с')
        for ax in axes:
            ax.grid(True)
            ax.legend()
        fig.savefig(base.with_suffix('.png'), dpi=150)
        plt.close(fig)
        message = ('No convergence' if self.convergence.result is None
                   else f'Convergence time: {self.convergence.result:.2f} s')
        self.get_logger().info(f'{message}; results: {base}')


def main(args=None):
    rclpy.init(args=args)
    node = Experiment()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    finally:
        node.drive(False)
        node.destroy_node()
        rclpy.shutdown()
    if node.failed:
        raise SystemExit(1)
