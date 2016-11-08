# RUN WITH "sudo /home/poppy/miniconda/bin/python test_joy2ergo.py"                                    

import os, sys
import pygame
import pygame.display
import time
from poppy.creatures import PoppyErgoJr


# INIT JOYSTICK
os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.display.init()
screen = pygame.display.set_mode((1,1))
pygame.joystick.init()
j = pygame.joystick.Joystick(0)
j.init()
print 'Initialized Joystick : %s' % j.get_name()

     
# INIT POPPY
poppy = PoppyErgoJr(camera="dummy")
poppy.compliant = False

# goto start position
for m,p in zip(poppy.motors, [0.0, -15.4, 35.34, -8.06, -15.69, 71.99]):
    m.goto_position(p, 1.)
time.sleep(1.)
print "Poppy started"

    
def servo_axis0(x, id):
    if x <= 1 and x >= -1:
        p = poppy.motors[id].goal_position
        if -180 < p+x < 180 : 
            poppy.motors[id].goto_position(p + 2*x, 0.1)

while True:
    try:
        time.sleep(0.01)
        pygame.event.get()
        x = j.get_axis(1)
        y = j.get_axis(0)
        servo_axis0(x, 0)
        servo_axis0(y, 1)
    except KeyboardInterrupt:
        poppy.compliant = True
        break

    
