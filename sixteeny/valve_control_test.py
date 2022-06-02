from sixteeny.controller.odor import OdorValveController
import time

with OdorValveController(minimum_delay=0.001) as odor:
    for i in range(16):
        odor.publish(i, [0, 1, 2])

# Diagnostic code
# with OdorValveController(minimum_delay=0.001) as odor:
#     for i in range(16):
#         odor.publish(i, [0, 0, 0])
#         time.sleep(5)
#         odor.publish(i, [1, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(5)
#         odor.publish(i, [0, 1, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(5)
#         odor.publish(i, [0, 0, 1])
#         time.sleep(10)
#     odor.publish(i, [0, 0, 0])


# import rclpy
# from rclpy.node import Node
# import time

# from y_arena_interfaces.msg import ArenaOdors

# rclpy.init()
# node = rclpy.create_node('arena_odors_publisher')
# publisher = node.create_publisher(ArenaOdors, 'arena_odors', 10)

# odors = [0,1,2]
# arena = 0

# msg = ArenaOdors()
# last_time = time.time()

# while True:
#     try:
#         current_time=time.time()
#         publisher.publish(msg)
#         msg.arena = arena
#         msg.odors = odors
#         arena = arena + 1
#         odors = [odors[2],odors[0],odors[1]]
#         if arena > 15:
#             arena = 0
#         print(msg.arena,msg.odors,current_time-last_time)
#         time.sleep(1)
#         last_time = current_time
#     except KeyboardInterrupt:
#         break

# node.destroy_node()
# rclpy.shutdown()

