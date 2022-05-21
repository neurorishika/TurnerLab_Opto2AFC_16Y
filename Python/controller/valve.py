from subprocess import call

call(["call","\yarena_ws\install\setup.bat"])

import rclpy
from rclpy.node import Node

from y_arena_interfaces.msg import ArenaOdors

class ArenaOdorsPublisher(Node):
    """
    A class to send odor signals to the Raspberry Pi 
    """
    def __init__(self):
        super().__init__('arena_odors_publisher')
        self.publisher = self.create_publisher(ArenaOdors, 'arena_odors', 10)
    
    def create_message(self,arena,odor_vector):
        """
        A method to create a message to send to the Raspberry Pi.
        """
        message = ArenaOdors()
        message.arena = arena
        message.odors = odor_vector
        return message
    
    def publish_message(self,arena,odor_vector):
        """
        A method to publish a message to the Raspberry Pi.
        """
        message = self.create_message(arena,odor_vector)
        self.publisher.publish(message)


