import rclpy
from rclpy.node import Node
import time
import serial
import numpy as np
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
        self.node = rclpy.create_node("arena_odors_publisher")
        self.publisher = self.node.create_publisher(ArenaOdors, "arena_odors", 10)
        time.sleep(1)
        print("ROS2 interface initialised")

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


class OdorSensorController(object):
    """
    A class to control the odor sensor using the Arduino Interface
    """

    def __init__(self, serial_port="COM4", baud_rate=115200):
        """
        Initialize the OdorSensorController class.
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.arduino_interface = None

    def init(self):
        """
        Initialize the Arduino interface.
        """
        self.arduino_interface = serial.Serial(self.serial_port, self.baud_rate)
        time.sleep(0.01)
        print("Arduino interface initialised")

    def __enter__(self):
        """|
        Enter the context manager.
        """
        self.init()
        return self

    def close(self):
        """
        Close the Arduino interface.
        """
        self.arduino_interface.close()

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Exit the context manager.
        """
        self.close()

    def read_sensor(self):
        """
        Read the sensor.
        """
        value = self.arduino_interface.readline().decode("utf-8")
        if value == "":
            return np.nan
        value = int(value.rstrip("\r\n"))
        return value
