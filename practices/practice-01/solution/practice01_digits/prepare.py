"""Replace the default turtle with two turtles at the digit origins."""

import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill, Spawn


class Prepare(Node):
    def __init__(self):
        super().__init__('prepare_digits')
        self.kill = self.create_client(Kill, '/kill')
        self.spawn = self.create_client(Spawn, '/spawn')

    def run(self):
        while rclpy.ok() and not self.kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim /kill')
        while rclpy.ok() and not self.spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim /spawn')

        future = self.kill.call_async(Kill.Request(name='turtle1'))
        rclpy.spin_until_future_complete(self, future)
        if future.exception():
            raise future.exception()

        for name, x in [('digit_left', 1.6), ('digit_right', 6.5)]:
            future = self.spawn.call_async(
                Spawn.Request(x=x, y=8.1, theta=0.0, name=name)
            )
            rclpy.spin_until_future_complete(self, future)
            if future.exception():
                raise future.exception()
            self.get_logger().info(f'Spawned {name}')


def main():
    rclpy.init()
    node = Prepare()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
