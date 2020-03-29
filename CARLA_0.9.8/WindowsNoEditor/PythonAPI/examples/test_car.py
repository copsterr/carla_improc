import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import pygame
import carla
import random
import time
import numpy as np
import cv2
import queue
from ds2_controller import DS2_Controller


""" Init Controller """
# init pygame
pygame.init()

# init joysticks functionality
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)  # get joystick
joystick.init()                         # init joystick
controller = DS2_Controller(joystick)



""" CONSTANTS """
IM_WIDTH = 640
IM_HEIGHT = 480

""" FNS """
def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    cv2.imshow("", i3)
    cv2.waitKey(1)
    return i3 / 255.0


actor_list = []

try:
    # connect to carla server
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)

    # get world and blueprints
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # set weather
    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        sun_altitude_angle=90.0)
    world.set_weather(weather)

    # get car blueprint
    car_bp = bp_lib.find("vehicle.tesla.cybertruck")
    spawn_point = random.choice(world.get_map().get_spawn_points())

    # spawn a car
    vehicle = world.try_spawn_actor(car_bp, spawn_point)
    actor_list.append(vehicle)



    # get rgb camera blueprint
    camera_bp = bp_lib.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    camera_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
    camera_bp.set_attribute("fov", "110")
    camera_bp.set_attribute("sensor_tick", "0.3")

    # adjust sensor location relative to vehicle
    camera_spawn_point = carla.Transform(carla.Location(x=4.0, z=1.5), carla.Rotation(pitch=-15.0))

    # spawn camera
    sensor = world.try_spawn_actor(camera_bp, camera_spawn_point, attach_to=vehicle)
    actor_list.append(sensor)

    image_queue = queue.Queue()
    sensor.listen(image_queue.put)

    # move the car
    # vehicle.apply_control(carla.VehicleControl(throttle=0.2, steer=0.0))

    done = False

    # GAME HERE
    while not done:
        world.tick()
        tstart = time.time()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        controller.update()
        if controller.cross:
            vehicle.apply_control(carla.VehicleControl(throttle=0.2, brake=0))
            print("Accelerating...")

        if controller.square:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            print("Brake...")

        img = image_queue.get()
        process_img(img)

        tprocess = time.time() - tstart
        # print(f"fps: {round(1/tprocess, 2)}")


finally:
    print("Destroying actors...")
    for actor in actor_list:
        actor.destroy()
    print("done.")
