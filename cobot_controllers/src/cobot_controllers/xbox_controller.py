from inputs import get_gamepad
import math
import threading

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.js_threshold = 0.1
        self.trig_threshold = 0.5
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.XDPad = 0
        self.yDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def reset(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.XDPad = 0
        self.yDPad = 0
        return 

    def read(self): # return the buttons/triggers that you care about in this methode
      return


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    #print(event.code)
                    js_val = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    #if js_val > self.js_threshold:
                    self.LeftJoystickY  = js_val
                elif event.code == 'ABS_X':
                    #print(event.code)
                    js_val = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    #if js_val > self.js_threshold:
                    self.LeftJoystickX  = js_val
                elif event.code == 'ABS_RY':
                    #print(event.code)
                    js_val = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    #if js_val > self.js_threshold:
                    self.RightJoystickY = js_val
                elif event.code == 'ABS_RX':
                    #print(event.code)
                    js_val = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                    #if js_val > self.js_threshold:
                    self.RightJoystickX  = js_val
                elif event.code == 'ABS_Z':
                    #print(event.code)
                    trig_val = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                    #if trig_val > self.trig_threshold:
                    self.LeftTrigger = trig_val
                elif event.code == 'ABS_RZ':
                    #print(event.code)
                    trig_val = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                    #if trig_val > self.trig_threshold:
                    self.RightTrigger = trig_val
                elif event.code == 'BTN_TL':
                    #print(event.code)
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    #print(event.code)
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    #print(event.code)
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    #print(event.code)
                    self.X = event.state
                elif event.code == 'BTN_WEST':
                    #print(event.code)
                    self.Y = event.state
                elif event.code == 'BTN_EAST':
                    #print(event.code)
                    self.B = event.state
                elif event.code == 'ABS_HAT0X':
                    #print(event.code)
                    self.XDPad =  event.state                
                elif event.code == 'ABS_HAT0Y':
                    #print(event.code)
                    self.YDPad =  event.state


                """
                elif event.code == 'BTN_THUMBL':
                    #print(event.code)
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    #print(event.code)
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    #print(event.code)
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    #print(event.code)
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    #print(event.code)
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    #print(event.code)
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    #print(event.code)
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    #print(event.code)
                    self.DownDPad = event.state"""
               

