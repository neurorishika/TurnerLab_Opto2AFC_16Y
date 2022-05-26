import serial
import time

def hex_to_rgb_intensity(hex_value):
    hex_value = hex_value.decode().lstrip('#')
    rgb = tuple(int(hex_value[i:i+2], 16) for i in (0, 2, 4))
    rgb_intensity = tuple(int(i/255*100) for i in rgb)
    return rgb_intensity

class LEDController(object):
    """
    A Class for the 16 Y LED controllers

    Variables:
        conns: list of serial connections (list of serial.Serial objects)
        arena_specs: dictionary of arena specifications (dict)
        initialized: whether the LED controller has been initialized (bool)
        ports: list of COM ports to connect to (list of str)
        baudrate: baudrate to use (int)
    
    Methods:
        __init__: 
            initialize the LED controller class
        setup_connections
            setup connections to the LED controllers
        close_connections:
            close connections to the LED controllers
        init:
            initialize the LED controller
        close:
            close the LED controller
        __enter__:
            initialize the LED controller using a context manager
        __exit__:
            close the LED controller using a context manager
        initialize_IR:
            initialize the IR backlight
        turn_on_backlight:
            turn on the backlight for all arenas
        turn_off_backlight:
            turn off the backlight for all arenas
        set_led_pulse_pattern:
            set the LED pulse pattern
        set_led_stimulus_intensity:
            set the LED stimulus intensity
        run_led_stimulus:
            run the LED stimulus
        reset_arena_led_stimuli:
            reset all LED stimuli in the arena
        reset_module_led_stimuli:
            reset all LED stimuli in the module
        led_stimulation:
            run the LED stimulation
    """

    def __init__(self,ports=['COM3', 'COM5', 'COM6', 'COM4'],baudrate=115200): 
        """
        Initialize the LED controller class
        
        Variables:
            ports: list of COM ports to connect to (list of str)
            baudrate: baudrate to use (int)
        """
        self.initialized=False
        self.ports = ports
        self.baudrate = baudrate
        self.arena_specs = {
            0: {'conn': 0, 'quadrant': b'0001'},
            1: {'conn': 0, 'quadrant': b'0010'},
            2: {'conn': 0, 'quadrant': b'1000'},
            3: {'conn': 0, 'quadrant': b'0100'},
            4: {'conn': 1, 'quadrant': b'0001'},
            5: {'conn': 1, 'quadrant': b'0010'},
            6: {'conn': 1, 'quadrant': b'1000'},
            7: {'conn': 1, 'quadrant': b'0100'},
            8: {'conn': 2, 'quadrant': b'0001'},
            9: {'conn': 2, 'quadrant': b'0010'},
            10: {'conn': 2, 'quadrant': b'1000'},
            11: {'conn': 2, 'quadrant': b'0100'},
            12: {'conn': 3, 'quadrant': b'0001'},
            13: {'conn': 3, 'quadrant': b'0010'},
            14: {'conn': 3, 'quadrant': b'1000'},
            15: {'conn': 3, 'quadrant': b'0100'}
        }
    
    def setup_connections(self):
        """
        Setup connections to the LED controllers using the COM ports

        Variables:
            ports: list of COM ports to connect to (list of str)
            baudrate: baudrate to use (int)

        Returns:
            conns: list of serial connections (list of serial.Serial objects)
        """
        self.conns = []
        for port in self.ports:
            try:
                conn = serial.Serial(port,self.baudrate)
                conn.write(b'RESET\r')
                conn.write(b'RED 0\r')
                conn.write(b'GRN 0\r')
                conn.write(b'BLU 0\r')
                conn.write(b'IR 0\r')
                conn.write(b'ON 0\r')
                self.conns.append(conn)
            except:
                print('Could not connect to ' + port)

    def close_connections(self):
        """
        Close connections to the LED controllers using the COM ports
        
        Variables:
            conns: list of serial connections (list of serial.Serial objects)
        """
        for conn in self.conns:
            conn.write(b'RESET\r')
            conn.close()
    
    def init(self):
        """
        Initialize the LED controllers 
        """
        self.setup_connections()
        self.initialized = True

    def __enter__(self):
        """
        Initialize the LED controller using a context manager
        """
        self.init()
        return self
    
    def close(self):
        """
        Close the LED controller
        """
        self.close_connections()
        self.initialized = False
    
    def __exit__(self, exc_type, exc_value, traceback):
        """
        Close the LED controller using a context manager
        """
        self.close()
    
    def initialize_IR(self,arena,ir_intensity):
        """
        Initialize the IR backlight

        Variables:
            arena: arena to initialize the backlight for (int)
            ir_intensity: intensity of IR LEDs (int)
        """
        assert ir_intensity >= 0 and ir_intensity <= 100, 'Invalid intensity'
        conn = self.conns[self.arena_specs[arena]['conn']]
        quadrant = self.arena_specs[arena]['quadrant']
        conn.write(b'IR ' + str(ir_intensity).encode() + b' ' + quadrant + b'\r')

    def turn_on_backlight(self,backlight_intensity):
        """
        Initialize the backlight for all arenas
        """
        # Initialize the LED controllers
        for arena in self.arena_specs.keys():
            self.initialize_IR(arena,backlight_intensity)

    def turn_off_backlight(self):
        """
        Turn off the backlight for all arenas
        """
        # Turn off the LED controllers
        for arena in self.arena_specs.keys():
            self.initialize_IR(arena,0)
    
    def set_led_pulse_pattern(self,arena,color,pulse_width,pulse_period,pulse_count,pulse_deadtime,pulse_delay,pulse_repeat):
        """
        Set a pulse pattern for the LED controllers
        
        Variables:
            arena: arena to set the pulse pattern for (int)
            color: color of the pulse pattern (byte str 'R' or 'G' or 'B' or '#<hex color>' / tuple of ints (0-100,0-100,0-100))
            pulse_width: width of the pulse pattern (int)
            pulse_period: period of the pulse pattern (int)
            pulse_count: number of pulses in the pulse pattern (int)
            pulse_deadtime: deadtime of the pulse pattern (int)
            pulse_delay: delay of the pulse pattern (int)
            pulse_repeat: number of times to repeat the pulse pattern (int)
        """
        assert pulse_width >= 0 and pulse_period >= 0 and pulse_count >= 0 and pulse_deadtime >= 0 and pulse_delay >= 0 and pulse_repeat >= 0, 'Invalid pulse parameters'
        assert color == b'R' or color == b'G' or color == b'B' or (len(color) ==7 and color[0] == 35) or (type(color) == tuple and len(color) == 3), 'Invalid color'
        conn = self.conns[self.arena_specs[arena]['conn']]
        if color == b'R' or color == b'G' or color == b'B':
            conn.write(b'PULSE ' + bytes(str(pulse_width),'utf-8')
                        + b' ' + bytes(str(pulse_period),'utf-8')
                        + b' ' + bytes(str(pulse_count),'utf-8')
                        + b' ' + bytes(str(pulse_deadtime),'utf-8')
                        + b' ' + bytes(str(pulse_delay),'utf-8')
                        + b' ' + bytes(str(pulse_repeat),'utf-8')
                        + b' ' + color + b'\r')
        else:
            conn.write(b'PULSE ' + bytes(str(pulse_width),'utf-8')
                        + b' ' + bytes(str(pulse_period),'utf-8')
                        + b' ' + bytes(str(pulse_count),'utf-8')
                        + b' ' + bytes(str(pulse_deadtime),'utf-8')
                        + b' ' + bytes(str(pulse_delay),'utf-8')
                        + b' ' + bytes(str(pulse_repeat),'utf-8')
                        + b'\r')

    def set_led_stimulus_intensity(self,arena,color,intensity):
        """
        Set the intensity of the LED stimulus

        Variables:
            arena: arena to set the intensity for (int)
            color: color of the LED stimulus (byte str 'R' or 'G' or 'B' or '#<hex color>' / tuple of ints (0-100,0-100,0-100))
            intensity: intensity of the LED stimulus (int 0-100)
        """
        assert intensity >= 0 and intensity <= 100, 'Invalid intensity'
        assert color == b'R' or color == b'G' or color == b'B' or (len(color) ==7 and color[0] == 35) or (type(color) == tuple and len(color) == 3), 'Invalid color'
        conn = self.conns[self.arena_specs[arena]['conn']]
        quadrant = self.arena_specs[arena]['quadrant']
        if color == b'R':
            conn.write(b'RED ' + str(intensity).encode() + b' 0 ' + quadrant + b'\r')
        elif color == b'G':
            conn.write(b'GRN ' + str(intensity).encode() + b' 0 ' + quadrant + b'\r')
        elif color == b'B':
            conn.write(b'BLU ' + str(intensity).encode() + b' 0 ' + quadrant + b'\r')
        else:
            if color[0] == 35:
                rgb_intensity = hex_to_rgb_intensity(color)
            else:
                rgb_intensity = color
            
            conn.write(b'RED ' + str(rgb_intensity[0]).encode() + b' 0 ' + quadrant + b'\r')
            conn.write(b'GRN ' + str(rgb_intensity[1]).encode() + b' 0 ' + quadrant + b'\r')
            conn.write(b'BLU ' + str(rgb_intensity[2]).encode() + b' 0 ' + quadrant + b'\r')

    def run_led_stimulus(self,arena,color=b'#000000'):
        """
        Run the LED stimulus
        
        Variables:
            arena: arena to run the LED stimulus for (int)
            color: color of the LED stimulus (byte str 'R' or 'G' or 'B' or '#<hex color>' / tuple of ints (0-100,0-100,0-100))
        """
        assert color == b'R' or color == b'G' or color == b'B' or (len(color) ==7 and color[0] == 35) or (type(color) == tuple and len(color) == 3), 'Invalid color'

        conn = self.conns[self.arena_specs[arena]['conn']]
        if color == b'R' or color == b'G' or color == b'B':
            conn.write(b'RUN ' + color + b'\r')
        else:
            conn.write(b'RUN\r')
    
    def stop_led_stimulus(self,arena,color=b'#000000'):
        """
        Stop the LED stimulus
        
        Variables:
            arena: arena to stop the LED stimulus for (int)
            color: color of the LED stimulus (byte str 'R' or 'G' or 'B' or '#<hex color>' / tuple of ints (0-100,0-100,0-100))
        """
        assert color == b'R' or color == b'G' or color == b'B' or (len(color) ==7 and color[0] == 35) or (type(color) == tuple and len(color) == 3), 'Invalid color'

        conn = self.conns[self.arena_specs[arena]['conn']]
        if color == b'R' or color == b'G' or color == b'B':
            conn.write(b'STOP ' + color + b'\r')
        else:
            conn.write(b'STOP\r')
    
    def pause_led_stimulus(self,arena):
        """
        Pause the LED stimulus
        
        Variables:
            arena: arena to pause the LED stimulus for (int)
        """
        conn = self.conns[self.arena_specs[arena]['conn']]
        conn.write(b'PAUSE \r')

    def reset_arena_led_stimuli(self,arena):
        """
        Reset all LED stimuli in the arena

        Variables:
            arena: arena to reset the LED stimuli for (int)
        """
        self.set_led_stimulus_intensity(arena,b'R',0)
        self.set_led_stimulus_intensity(arena,b'G',0)
        self.set_led_stimulus_intensity(arena,b'B',0)
    
    def reset_module_led_stimuli(self,arena):
        """
        Reset all LED stimuli in the module

        Variables:
            arena: arena to reset the LED stimuli for (int)
        """
        conn = self.conns[self.arena_specs[arena]['conn']]
        conn.write(b'RED 0\r')
        conn.write(b'GRN 0\r')
        conn.write(b'BLU 0\r')

    def led_stimulation(self,arena,color,intensity,pulse_width,pulse_period,pulse_count,pulse_deadtime,pulse_delay,pulse_repeat,debug=False):
        """
        Run the LED stimulus

        Variables:
            arena: arena to run the LED stimulus for (int)
            color: color of the LED stimulus (byte str: 'R' or 'G' or 'B' or '#<hex color>' / tuple of ints (0-100,0-100,0-100))
            intensity: intensity of the LED stimulus (int)
            pulse_width: width of the pulse pattern (int)
            pulse_period: period of the pulse pattern (int)
            pulse_count: number of pulses in the pulse pattern (int)
            pulse_deadtime: deadtime of the pulse pattern (int)
            pulse_delay: delay of the pulse pattern (int)
            pulse_repeat: number of times to repeat the pulse pattern (int)
        """
        assert intensity >= 0 and intensity <= 100, 'Invalid intensity'
        assert color == b'R' or color == b'G' or color == b'B' or (len(color) ==7 and color[0] == 35) or (type(color) == tuple and len(color) == 3), 'Invalid color'
        assert pulse_width >= 0 and pulse_period >= 0 and pulse_count >= 0 and pulse_deadtime >= 0 and pulse_delay >= 0 and pulse_repeat >= 0, 'Invalid pulse parameters'
        
        if debug:
            print('Arena: ' + str(arena), end=' ')
            print('Pulse ' + str(pulse_width) + ' ' + str(pulse_period) + ' ' + str(pulse_count) + ' ' + str(pulse_deadtime) + ' ' + str(pulse_delay) + ' ' + str(pulse_repeat) + ' ' + color.decode())

        self.set_led_pulse_pattern(arena,color,pulse_width,pulse_period,pulse_count,pulse_deadtime,pulse_delay,pulse_repeat)
        self.reset_module_led_stimuli(arena)
        self.set_led_stimulus_intensity(arena,color,intensity)
        self.run_led_stimulus(arena)
        # time.sleep(self.get_stimulus_duration(pulse_width,pulse_period,pulse_count,pulse_deadtime,pulse_delay,pulse_repeat))

    def run_json(self,arena,json_dict,debug=False):
        """
        Run the LED stimulus from a JSON dictionary

        Variables:
            arena: arena to run the LED stimulus for (int)
            json_dict: JSON data to run the LED stimulus for (dict)
        """
        assert type(json_dict) == dict, 'Invalid JSON data'
        
        self.led_stimulation(arena, json_dict['color'].encode(), json_dict['intensity'], json_dict['pulse_width'], json_dict['pulse_period'], json_dict['pulse_count'], json_dict['pulse_deadtime'], json_dict['pulse_delay'], json_dict['pulse_repeat'], debug)

    def get_stimulus_duration(self,pulse_width,pulse_period,pulse_count,pulse_deadtime,pulse_delay,pulse_repeat):
        """
        Get the length of the LED stimulus in seconds

        Variables:
            pulse_width: width of the pulse pattern (int)
            pulse_period: period of the pulse pattern (int)
            pulse_count: number of pulses in the pulse pattern (int)
            pulse_deadtime: deadtime of the pulse pattern (int)
            pulse_delay: delay of the pulse pattern (int)
            pulse_repeat: number of times to repeat the pulse pattern (int)
        """
        return pulse_delay*1000 + pulse_repeat*(pulse_deadtime + pulse_count*pulse_period)
    
    ### IMPLEMENT MULTIPLEXED LED STIMULUS ###
