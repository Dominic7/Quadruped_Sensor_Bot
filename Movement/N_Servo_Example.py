#//================================================ file = walkingPython.py ======
#//= Notes:                                                                      =
#//=      - Used to test N number of servos with a RPI and Adafruit PWM driver   =
#//=            board                                                            =
#//=---------------------------------------------------------------------------- =
#//=  Execute: python N_Servo_Example.py                                         = 
#//=---------------------------------------------------------------------------- =
#//=  Author: Dominic Cox                                                        =
#//=       Email: dominicacox@gmail.com                                          =
#//===============================================================================
from Adafruit_PWM_Servo_Driver import PWM
import time


pwm = PWM(0x40) #PWM(0x40, debug=True)
pwm.setPWMFreq(50) #Ms

servoMin = 115  # Min pulse length out of 4096
servoMax = 490  # Max pulse length out of 4096

NUM_SERVOS = 5 # servos plugged in starting at channel0

def setAngle(channel, angle, delta=170):
    '''
        Sets angle of servo (approximate).
        channel: Channel servo is attached to (0-15)
        angle:   off of center, ( -80 through 80)
        delta:   Angle changed from past position. Used to calculate delay
    '''
    delay = max(delta * 0.003, 0.03)            # calculate delay
    zero = (servoMin + servoMax) / 2            # half-way == 0 degrees
    pulse_width = zero - servoMin               # maximum pulse to either side
    pulse = zero + (pulse_width * angle / 80)
    pwm.setPWM(channel, 0, int(pulse))
    time.sleep(delay)                           # sleep to give the servo time to do its thing

    
count = -80 #start angle
for i in range(160):
    for j in range(NUM_SERVOS):
        if count is -80:
            setAngle(j,count)
        else:
            setAngle(j, count, count-1)
