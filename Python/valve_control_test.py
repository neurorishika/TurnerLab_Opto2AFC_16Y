import rclpy
from rclpy.node import Node
import time

from y_arena_interfaces.msg import ArenaOdors

rclpy.init()
node = rclpy.create_node('arena_odors_publisher')
publisher = node.create_publisher(ArenaOdors, 'arena_odors', 10)

msg = ArenaOdors()
msg.arena = 0
msg.odors = [0,1,2]
last_time = time.time()

while True:
    try:
        current_time=time.time()
        publisher.publish(msg)
        rclpy.spin_once(node)
        msg.arena = msg.arena + 1
        msg.odors = [msg.odors[1],msg.odors[2],msg.odors[0]]
        if msg.arena > 16:
            msg.arena = 0
        print(msg.arena,msg.odors,current_time-last_time)
        last_time = current_time
    except KeyboardInterrupt:
        break

node.destroy_node()
rclpy.shutdown()

