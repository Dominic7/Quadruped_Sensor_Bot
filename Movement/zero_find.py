#//================================================ file = zero_find.py ==========
#//= Notes:                                                                      =
#//=      - Used to find the center(zero) for N number of servos at once         =
#//=---------------------------------------------------------------------------- =
#//=  Execute: python zero_find.py                                               = 
#//=---------------------------------------------------------------------------- =
#//=  Author: Dominic Cox                                                        =
#//=       Email: dominicacox@gmail.com                                          =
#//===============================================================================
from Adafruit_PWM_Servo_Driver import PWM
import time


pwm = PWM(0x40) #PWM(0x40, debug=True)
pwm.setPWMFreq(50)

servoMin = 115  # Min pulse length out of 4096
servoMax = 490  # Max pulse length out of 4096

NUM_SERVOS = 1 # servos plugged in starting at channel0

def setAngle(channel, angle, delta=170):
    '''
        Sets angle of servo (approximate).
        channel: Channel servo is attached to (0-15)
        angle:   off of center, ( -80 through 80)
        delta:   Angle changed from past position. Used to calculate delay
    '''
    delay = max(delta * 0.003, 0.03)          # calculate delay
    zero = (servoMin + servoMax) / 2          # half-way == 0 degrees
    pulse_width = zero - servoMin             # maximum pulse to either side
    pulse = zero + (pulse_width * angle / 80)
    pwm.setPWM(channel, 0, int(pulse))
    time.sleep(delay)                         # sleep to give the servo time to do its thing

    

for i in range(NUM_SERVOS):
    setAngle(i, 0)