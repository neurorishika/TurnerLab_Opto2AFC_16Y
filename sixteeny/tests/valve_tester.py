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

