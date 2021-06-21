import numpy as np
import time
import board
from adafruit_motorkit import MotorKit
import busio
import board
import digitalio
from smbus import SMBus

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

#I2C initialization
i2c = busio.I2C(board.SCL, board.SDA, frequency = 400000)

def i2c_test():
    bus1 = SMBus(1)
    bus4 = SMBus(4)

    out = bus1.read_byte_data(0x4A,0x1)
    print(out)


try:
    imu = BNO08X_I2C(i2c, address=0x4A)
    imu.enable_feature(BNO_REPORT_ACCELEROMETER)
    imu.enable_feature(BNO_REPORT_GYROSCOPE)
    imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    print('[CHECK     ]  [IMU - BNO08X]')
except Exception as e:
    print('[     ERROR]  [IMU - BNO08X]',e)

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
        self.gain = 7 # 12V / 1.7 rad = 7 
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
            accel_x, accel_y, accel_z = imu.acceleration
            gyro_x, gyro_y, gyro_z = imu.gyro
            quat_i, quat_j, quat_k, quat_real = imu.quaternion
            time.sleep(0.5)

            print('gyro: %0.3f'%(gyro_z))
            state = [quat_k, gyro_z, 0, self.ref - quat_k]

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
        print('k_motor: %0.3f'%(self.k_motor) + '\n') 
        u = self.saturate_handle(u)

        state[2] = u * self.k_motor
        self.update_state_collection(state)
              
        try:
            self.kit.motor1.throttle = u

        except Exception as e:
            self.turn_off()
            print('ERROR in control: ', e)