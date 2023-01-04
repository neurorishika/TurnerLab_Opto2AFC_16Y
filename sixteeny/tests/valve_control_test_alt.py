from sixteeny.controller.odor import OdorValveController
import time

with OdorValveController(minimum_delay=0.05) as odor:
    print("Starting valve test through rapid flips")
    for i in range(16):
        for z in [[0,0,0],[0,0,1],[0,1,0],[1,0,0],[0,0,2],[0,2,0],[2,0,0],[0,0,0]]:
            odor.publish(i, z)
            time.sleep(0.1)
    print("Did all valves flip accurately? (y/n)")
    if not (input() == "y" or input() == "Y"):
        print("Valve test failed. Please check the valves and try again.")
        exit()
    print("Starting valve pressure tests.")
    while True:
        try:
            for code,valve in enumerate(["Air","Odor 1","Odor 2"]):
                print("Directing all valves to " + valve + " for 30 seconds.")
                for i in range(16):
                    odor.publish(i, [code, code, code])
                    time.sleep(0.1)
                for i in range(30):
                    print("Waiting for pressure to stabilize for 30 seconds. Time remaining: " + str(30 - i) + " seconds.", end="\r")
                    time.sleep(1)
                print()
                print("Release all valves, listen for pressure release.")
                for i in range(16):
                    odor.publish(i, [0, 1, 2])
                    time.sleep(0.5)
                print("Did any of the valves release pressure? (y/n)")
                if not (input() == "n" or input() == "N"):
                    print("Valve test failed. " + valve + " is blocked in one or more pipes. Please check the valves that released pressure and try again.")
                    exit()
                print("Press enter to continue.")
                input()
        except KeyboardInterrupt:
            print("Valve test complete.")
            break

        

        # odor.publish(i, [0, 0, 0])
        # time.sleep(0.1)

# # Diagnostic code
# with OdorValveController(minimum_delay=0.001) as odor:
#     for i in range(16):
#         odor.publish(i, [1, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [2, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 1, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 2, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 1])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 2])
#         time.sleep(10)
#         odor.publish(i, [0, 0, 0])
#         time.sleep(10)
        


# import rclpy
# from rclpy.node import Node
# import time

# from y_arena_interfaces.msg import ArenaOdors

# rclpy.init()
# node = rclpy.create_node('arena_odors_publisher')
# publisher = node.create_publisher(ArenaOdors, 'arena_odors', 1)

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
#         time.sleep(10)
#         last_time = current_time
#     except KeyboardInterrupt:
#         break

# node.destroy_node()
# rclpy.shutdown()

