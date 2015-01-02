#//================================================ file = Quadruped.py =====
#//= Notes:
#//=      Legs are numbered from 0 to 3 where leg[0] is linked to servo ids 0-2, =
#         leg[1] linked to 3-5, leg[2] linked to 6-8, and leg[3] linked to 9-11  =
#                                                                                =
#//=      stand() --> balance mode                                               = 
#//=      relax() --> should be off mode(still need to verify)                   = 
#//=---------------------------------------------------------------------------- =
#//=  To Do: Still needs:                                                        =
#                 - bodyIK                                                       =
#                 - controller support                                           =
#                 - sensor support                                               =                                            =
#//=---------------------------------------------------------------------------- =
#//=  Execute: python Quadruped.py                                               = 
#//=---------------------------------------------------------------------------- =
#//=  Author: Dominic Cox                                                        =
#       Email: dominicacox@gmail.com                                             =
#//=---------------------------------------------------------------------------- =
#//=  History:                                                                   =
                #12-18-14 Creation                                               =
#//===============================================================================
#//----- Imports -----------------------------------------------------------------

from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO
import time
import math

#//----Globals--------------------------------------------------------------------

pwm = PWM(0x40) #PWM(0x40, debug=True)

servoMin = 115  # Min pulse length out of 4096
servoMax = 490  # Max pulse length out of 4096

angleMin = -80
angleMax = 80
angleCenter = 0


num_servos = 3

num_legs = 4

#//===========================================================================
#//=  Program Classes                                                        =
#//===========================================================================


class point:
    '''defines a 3d point'''
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class joint:
    '''joint defines one servo with software, min, max and home degree values'''
    def __init__(self, id, home=0, min=servoMin, max=servoMax):
        self.id = id
        self.pulse_min = min
        self.pulse_max = max
        self.home_a = home
        self.last_angle = self.home_a

    def set_angle(self, angle):
        '''sets angle of servo'''
        delay = max(self.last_angle * 0.0018333, 0.0183)        # calculate delay
        zero = (self.pulse_min + self.pulse_max) / 2            # half-way == 0 degrees
        pulse_width = zero - self.pulse_min                     # maximum pulse to either side
        pulse = zero + (pulse_width * angle / 80)
        pwm.setPWM(self.id, 0, int(pulse))
        self.last_angle = angle
        time.sleep(delay)                                       # sleep to give the servo time to do its thin

    def go_home(self):
        self.set_angle(self.home_a)



class leg:
    '''defines robot leg with 3 joints'''
    def __init__(self, id, home_pt,coxA, femur, tibia, coxA_j=None, femur_j=None, tibia_j=None):
        '''id --> leg id used for gait cycle
           home_pt --> point in x,y,z space where the coxA leg joint servo is
           coxA --> length from coxA joint(servo) to femur joint(servo)
           femur --> length from femur joint(servo) to tibia joint(servo)
           tibia --> tangent length from tibia joint(servo) to tip of tibia(where it makes contact with the ground
           coxa_j, femur_j, tibia_j --> joint instanaces of the respective joints in the leg
        '''

        self.id = id                #leg id - used for gait cycle
        self.home_pt = home_pt       #home point of the endpoint of the leg in x,y,z space
        self.curr_loc = self.home_pt #current location of endpoint of the leg in x,y,z space -- starts at home point

        #----length of each leg section
        self.coxA_len = coxA
        self.femur_len = femur
        self.tibia_len = tibia

        #----leg joint assignment 
        self.coxA_joint = coxA_j
        self.femur_joint = femur_j
        self.tibia_joint = tibia_j

    def home(self):
        '''sets all joints(servos) to home position'''
        self.coxA_joint.go_home()
        self.femur_joint.go_home()
        self.tibia_joint.go_home()


    def move(self, target):
        '''IK for new leg angles for the target '''
        new_coxA_a = math.degrees(math.atan2((target.x - self.curr_loc.x), (target.y - self.curr_loc.y)))

        len_xy = math.sqrt(math.pow(target.x - self.curr_loc.x,2) + math.pow(target.y - self.curr_loc.y, 2))

        len_z = math.sqrt(math.pow(target.z - self.curr_loc.z, 2) + math.pow(len_xy-self.coxA_len, 2)) #may need to switch target.z and curr_loc z if getting wrong values -- 
        #^not really sure why this needs to be calculated this way....


        new_femur_a = math.degrees(math.acos((target.z - self.curr_loc.z)/len_z)) #first half of the ik equation
        new_femur_a += math.degrees(math.acos(((math.pow(self.tibia_len,2) - math.pow(self.femur_len,2) - math.pow(len_z,2))/(-2*self.femur_len*len_z))))

        new_tibia_a = math.degrees(math.acos(((math.pow(len_z,2)-math.pow(self.tibia_len,2)-math.pow(self.femur_len))/(-2*self.tibia_len*self.femur_len))))

        self.move_joint_angles(new_coxA_a, new_femur_a, new_tibia_a)
        self.curr_loc = target

    def move_joint_angles(self, coxA_a, femur_a, tibia_a):
        ''' move servoes to passed angles'''
        self.coxA_joint.set_angle(coxA_a)
        self.femur_joint.set_angle(femur_a)
        self.tibia_joint.set_angle(tibia_a)


#//===========================================================================
#//=  Main Class                                                             =
#//===========================================================================


class quadruped:
    '''defines a quadruped with 4 legs and 3 joints(servos) per leg'''

    def __init__(self, body_x, body_y, body_z ,leg_angle_offset, coxA_home, femur_home, tibia_home, coxA_len, femur_len, tibia_len):
        '''body_x, body_y, & body_z are in mm and leg_angle_offset is in degrees
           body_x --> length of body treated as x plane
           body_y --> length of body treated as y plane
           body_z --> height of bot from ground to bottom of bot
           leg_angle_offset --> angle of leg from body; **should be same for every leg and should be same for either side of leg
        '''
        self.offsetX = float(body_x/2)
        self.offsetY = float(body_y/2)
        self.offsetZ = body_z
        self.offsetA = leg_angle_offset

        pwm.setPWMFreq(50)
        
        self.legs = []
        servo_id = 0
        
        

        for i in range(4):      #legs 0 & 2 are the front --- legs 1 & 3 are the back
            
            #need to verify the following
            if i % 2:
                temp_coxA = joint(servo_id, coxA_home)
                temp_femur = joint(servo_id + 1, femur_home)
                temp_tibia = joint(servo_id + 2, tibia_home)
            else:
                temp_coxA = joint(servo_id, -coxA_home)
                temp_femur = joint(servo_id + 1, -femur_home)
                temp_tibia = joint(servo_id + 2, -tibia_home)

            if i < 2:
                offsetX = -self.offsetX
            else:
                offsetX = self.offsetX
            if i % 2 is 1:
                offsetY = -self.offsetY
            else:
                offsetY = self.offsetY

            temp_leg = leg(i, point(offsetX, offsetY, self.offsetZ), coxA_len, femur_len, tibia_len, temp_coxA, temp_femur, temp_tibia)

            self.legs.append(temp_leg)
            servo_id += 3

    def stand(self):
        self.legs[0].home()
        self.legs[3].home()
        self.legs[2].home()
        self.legs[1].home()
    
    def relax(self):
        pwm.setAllPWM(0,0)

    def forward(self, steps):
        crawl_gait = {}
        crawl_gait[0] = (0,-25,70)
        crawl_gait[1] = (-5,-25,70)
        crawl_gait[2] = (-10,-25,70)
        crawl_gait[3] = (-15,-25,70)
        crawl_gait[4] = (-20,-10,80)
        
        count = 0
        start = -20
        for i in range(5,20):
            if count % 3 is 2:
                start += 2
            else:
                start += 1
            crawl_gait[i] = (start, -10, 80)
            count += 1

        leg_gait_pos = [5,10,15,0]
        for s in range(steps):
            for i in range(20): #20 step gait cycle
                for j in range(4): #4 legs
                    angles = crawl_gait[leg_gait_pos[j]]
                    leg_gait_pos[j] = (leg_gait_pos[j]+1)%20
                    self.legs[j].move_joint_angles(angles[0],angles[1],angles[2])

#//===========================================================================
#//=  Program init                                                           =
#//===========================================================================


if __name__ == "__main__":
    #setting OE pin on driver board HIGH to make sure no movement while initializing everything
    OE_pin = 11
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(OE_pin, GPIO.OUT)
    GPIO.output(OE_pin, 1)
    body_x_length = 0
    body_y_length = 0
    body_z_height = 0
    leg_angle_offset = 45

    coxA_home = 0
    femur_home = -10
    tibia_home = 80

    coxA_len = 0
    femur_len = 0
    tibia_len = 0

    robot = quadruped(body_x_length, body_y_length, body_z_height, leg_angle_offset, coxA_home, femur_home, tibia_home, coxA_len, femur_len, tibia_len)
    start = raw_input("Please enter 1 to start bot:")
    if int(start) is 1:
        #setting back to LOW to allow signals to pass
        GPIO.output(OE_pin,0)
        robot.stand()
        robot.forward(1)
        #setting back to HIGH since the driver board is still functional even after sketch is terminated 
        GPIO.output(OE_pin, 1)