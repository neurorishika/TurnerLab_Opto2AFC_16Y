import rclpy
from rclpy.node import Node
import time
from y_arena_interfaces.msg import ArenaOdors

class OdorValveController(object):
    """
    A class to control the valve using the Raspberry Pi ROS2 interface.
    """
    def __init__(self, minimum_delay=0.1):
        """
        Initialize the OdorValveController class.
        """
        self.node = None
        self.publisher = None
        self.minimum_delay = minimum_delay
    
    def init(self, *args, **kwargs):
        """
        Initialize the ROS2 interface.
        """
        rclpy.init(*args, **kwargs)
        self.node = rclpy.create_node('arena_odors_publisher')
        self.publisher = self.node.create_publisher(ArenaOdors, 'arena_odors', 10)
        time.sleep(1)
    
    def __enter__(self):
        """
        Enter the context manager.
        """
        self.init()
        return self
    
    def close(self):
        """
        Close the ROS2 interface.
        """
        self.node.destroy_node()
        rclpy.shutdown()
    
    def __exit__(self, exc_type, exc_value, traceback):
        """
        Exit the context manager.
        """
        self.close()
    
    def publish(self, arena, odor_vector):
        """
        Publish the odors and arena.

        Variables:
            arena (int): The arena number.
            odor vector (list): The odors identifiers for arms [0/1/2, 0/1/2, 0/1/2].
        """
        msg = self.create_msg(arena, odor_vector)
        self.publisher.publish(msg)
        time.sleep(self.minimum_delay)
        print(msg)
    
    def create_msg(self, arena, odor_vector):
        """
        Create the message.

        Variables:
            arena (int): The arena number.
            odor vector (list): The odors identifiers for arms [0/1/2, 0/1/2, 0/1/2].
        """
        msg = ArenaOdors()
        msg.arena = arena
        msg.odors = odor_vector
        return msg

