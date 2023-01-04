from led import LEDController
import time

RED_INTENSITY = 100
GREEN_INTENSITY = 100
PULSE_WIDTH = 100
PULSE_PERIOD = 200
PULSE_COUNT = 1
PULSE_DEADTIME = 0
PULSE_DELAY = 0
PULSE_REPEAT = 1

with LEDController(ports=['COM13','COM5','COM12','COM4']) as LC: # put in the correct ports here
    while True:
        LC.reset_accumulated_led_stimulus()

        for arena in range(16):
            LC.accumulate_led_stimulus(arena,b'R',RED_INTENSITY,PULSE_WIDTH,PULSE_PERIOD,PULSE_COUNT,PULSE_DEADTIME,PULSE_DELAY,PULSE_REPEAT,debug_mode=False)
            ## Turn on for multicolor stimulation
            # LC.accumulate_led_stimulus(arena,b'G',GREEN_INTENSITY,PULSE_WIDTH,PULSE_PERIOD,PULSE_COUNT,PULSE_DEADTIME,PULSE_DELAY,PULSE_REPEAT,debug_mode=False)
        LC.run_accumulated_led_stimulus()
        time.sleep(0.5)
        
        ## Turn on only if you want to reset the LED controller
        for i in range(4):
            LC.conns[i].write(b'RESET\r')
        time.sleep(1)

        LC.reset_accumulated_led_stimulus()
        for arena in range(16):
            LC.accumulate_led_stimulus(arena,b'R',RED_INTENSITY,PULSE_WIDTH//2,PULSE_PERIOD//2,PULSE_COUNT*2,PULSE_DEADTIME,PULSE_DELAY,PULSE_REPEAT,debug_mode=False)
            ## Turn on for multicolor stimulation
            # LC.accumulate_led_stimulus(arena,b'G',GREEN_INTENSITY,PULSE_WIDTH//2,PULSE_PERIOD//2,PULSE_COUNT*2,PULSE_DEADTIME,PULSE_DELAY,PULSE_REPEAT,debug_mode=False)
        LC.run_accumulated_led_stimulus()
        time.sleep(0.5)

        ## Turn on only if you want to reset the LED controller
        for i in range(4):
            LC.conns[i].write(b'RESET\r')
        time.sleep(1)
    