import pygame
import time
from ds2_controller import *

# init pygame
pygame.init()

# init joysticks functionality
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)  # get joystick
joystick.init()                         # init joystick

# create a controller object
controller = DS2_Controller(joystick)

done = False

# forever loop
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    controller.update()  # update controller to get all buttons' value
    controller.debug()   # prints all values in the terminal screen
    time.sleep(0.1)      # delay for 0.1 seconds to prevent program from running too fast

    # press Ctrl+C to terminate the program
