import numpy as np
import time
import board
from adafruit_motorkit import MotorKit
import busio
import board
import digitalio
import serial
from adafruit_bno08x_rvc import BNO08x_RVC

def uart_test():
    print('hello')
    uart = serial.Serial("/dev/serial0", 115200)
    rvc = BNO08x_RVC(uart)
    i = 0
    while i < 100:
        uart = serial.Serial("/dev/serial0", 115200)
        rvc = BNO08x_RVC(uart)
        print('in loop...')
        yaw, pitch, roll, x_accel, y_accel, z_accel = rvc.heading
        print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
        print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
        print("")
        time.sleep(0.1)
        i += 1
        
try:
    kit = MotorKit()
    print('[CHECK     ]  [MOTOR DRIVER]')
except Exception as e:
    print('[     ERROR]  [MOTOR DRIVER]',e)

class controller:
    
    def __init__(self):
        
        self.state = []
        self.state_collection = {'x1':[], 'x2':[],'x3':[],'x4':[], 'u':[]}
        self.ref = float(0)
        self.gain = 1.2 # 12V / 10 deg/s = 1.2 
        self.k_motor = 10 #?
        self.kit = kit
        self.saturated_voltage = 12 #V

    def turn_off(self):

        print('TURNING OFF')
        self.kit.motor1.throttle = 0

    def deinit(self):

        pass

    #Read current sensor values
    def get_state(self):
        
        try:
            uart = serial.Serial("/dev/serial0", 115200)
            rvc = BNO08x_RVC(uart)
            gyro_z, gyro_x, gyro_y, x_accel, y_accel, z_accel = rvc.heading
            time.sleep(0.5)

            print('gyro: %0.3f'%(gyro_z))
            state = [0, gyro_z, 0, self.ref - 0]

            return state

        except Exception as e:
            self.turn_off()
            print('ERROR in get_state: ',e)
            
    #Handle situation where control input is too large for max motor voltage 
    def saturate_handle(self, signal):

        if signal > 1:
            print('Saturated Signal...Desaturating from %0.3f V to 12V' %(signal*self.saturated_voltage))
            return 1 

        elif signal < -1:
            print('Saturated Signal...Desaturating from %0.3f V to -12V' %(signal*self.saturated_voltage))
            return -1

        else:
            return signal

    #Store state values into lists
    def update_state_collection(self, state):

        self.state_collection['x1'].append(state[0])
        self.state_collection['x2'].append(state[1])
        self.state_collection['x3'].append(state[2])
        self.state_collection['x4'].append(state[3])

    #Apply control law and actuate motor
    def control(self, state):
        
        u = self.gain * state[1] / self.saturated_voltage
        print('u: %0.3f'%(u)) 
        u = self.saturate_handle(u)

        state[2] = u * self.k_motor
        self.update_state_collection(state)
              
        try:
            self.kit.motor1.throttle = u

        except Exception as e:
            self.turn_off()
            print('ERROR in control: ', e)
