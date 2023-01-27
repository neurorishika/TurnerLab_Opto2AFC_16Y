import rclpy
from rclpy.node import Node

from y_arena_interfaces.msg import ArenaOdors


class ArenaOdorsPublisher(Node):

    ARENA_COUNT = 16

    def __init__(self):
        super().__init__('arena_odors_publisher')
        self.publisher_ = self.create_publisher(ArenaOdors, '/arena_odors', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self._arena = 0
        self._iteration = 0

    def timer_callback(self):
        msg = ArenaOdors()
        msg.arena = self._arena
        if self._arena + 1 == self.ARENA_COUNT:
            self._iteration += 1
        # set message odors
        if self._iteration == 0:
            msg.odors = [0, 0, 0]
        elif self._iteration == 1:
            msg.odors = [0, 1, 2]
        elif self._iteration == 2:
            msg.odors = [0, 2, 1]
        elif self._iteration == 3:
            msg.odors = [1, 0, 2]
        elif self._iteration == 4:
            msg.odors = [2, 0, 1]
        elif self._iteration == 5:
            msg.odors = [1, 2, 0]
        elif self._iteration == 6:
            msg.odors = [2, 1, 0]
        else:
            msg.odors = [0, 0, 0]
        self.publisher_.publish(msg)
        self._arena = (self._arena + 1) % self.ARENA_COUNT


def main(args=None):
    rclpy.init(args=args)

    arena_odors_publisher = ArenaOdorsPublisher()

    rclpy.spin(arena_odors_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arena_odors_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()