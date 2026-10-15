"""Pose based seven-segment digit drawing using one turtle per process."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


# Segment ends in local coordinates; digit is 2.5 by 5 turtlesim units.
SEGMENTS = {
    'a': ((0, 5), (2.5, 5)),
    'b': ((2.5, 5), (2.5, 2.5)),
    'c': ((2.5, 2.5), (2.5, 0)),
    'd': ((2.5, 0), (0, 0)),
    'e': ((0, 0), (0, 2.5)),
    'f': ((0, 2.5), (0, 5)),
    'g': ((0, 2.5), (2.5, 2.5)),
}

DIGITS = {
    '0': 'abcdef', '1': 'bc', '2': 'abged', '3': 'abgcd',
    '4': 'fgbc', '5': 'afgcd', '6': 'afgecd', '7': 'abc',
    '8': 'abcdefg', '9': 'abfgcd',
}


def angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


class DigitDrawer(Node):
    def __init__(self):
        super().__init__('digit_drawer')
        self.declare_parameter('turtle', 'digit_left')
        self.declare_parameter('digit', 0)
        self.declare_parameter('origin_x', 1.6)
        self.declare_parameter('origin_y', 3.1)
        turtle = str(self.get_parameter('turtle').value)
        digit = str(self.get_parameter('digit').value)
        if digit not in DIGITS:
            raise ValueError(f'Unsupported digit: {digit}')
        x0 = float(self.get_parameter('origin_x').value)
        y0 = float(self.get_parameter('origin_y').value)

        self.actions = []
        for segment in DIGITS[digit]:
            start, end = SEGMENTS[segment]
            self.actions.extend([
                ('pen', True),
                ('move', (x0 + start[0], y0 + start[1])),
                ('pen', False),
                ('move', (x0 + end[0], y0 + end[1])),
            ])
        self.actions.append(('pen', True))
        self.index = 0
        self.reported_done = False
        self.pose = None
        self.pen_future = None
        self.pen_client = self.create_client(SetPen, f'/{turtle}/set_pen')
        self.publisher = self.create_publisher(Twist, f'/{turtle}/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, f'/{turtle}/pose', self.on_pose, 10
        )
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Drawing {digit} with {turtle}')

    def on_pose(self, pose):
        self.pose = pose

    def tick(self):
        command = Twist()
        if self.pose is None or self.index >= len(self.actions):
            if self.index >= len(self.actions) and not self.reported_done:
                self.get_logger().info('Digit complete; publishing zero velocity')
                self.reported_done = True
            self.publisher.publish(command)
            return

        kind, value = self.actions[self.index]
        if kind == 'pen':
            self.publisher.publish(command)
            if self.pen_future is None:
                if self.pen_client.service_is_ready():
                    self.pen_future = self.pen_client.call_async(
                        SetPen.Request(r=220, g=230, b=255, width=3, off=int(value))
                    )
            elif self.pen_future.done():
                if self.pen_future.exception():
                    raise self.pen_future.exception()
                self.pen_future = None
                self.index += 1
            return

        target_x, target_y = value
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.hypot(dx, dy)
        if distance < 0.065:
            self.index += 1
        else:
            error = angle_error(math.atan2(dy, dx), self.pose.theta)
            if abs(error) > 0.035:
                command.angular.z = max(-2.0, min(2.0, 4.0 * error))
            else:
                command.linear.x = min(1.4, max(0.18, 2.0 * distance))
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = DigitDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publisher.publish(Twist())
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # A second signal can arrive while launch is stopping the node.
            pass


if __name__ == '__main__':
    main()
