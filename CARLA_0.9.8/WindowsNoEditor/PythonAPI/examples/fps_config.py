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

import carla
import random
import time
import numpy as np
import cv2


client = carla.Client("localhost", 2000)
client.set_timeout(5.0)

world = client.get_world()

settings = world.get_settings()
settings.fixed_delta_seconds = 0.1
settings.synchronous_mode = True

world.apply_settings(settings)
print("Time-Step/Sync-Mode config completed.")