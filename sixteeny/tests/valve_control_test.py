from sixteeny.controller.odor import OdorValveController
import time

with OdorValveController(minimum_delay=0.05) as odor:
    print("Starting valve test through rapid flips")
    for i in range(16):
        odor.publish(i, [0,0,0])
        time.sleep(0.1)
