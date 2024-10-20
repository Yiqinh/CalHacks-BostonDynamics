import os
import time
from spot_controller_yiqin_dev import SpotController
import numpy as np
import cv2 as cv
import math

ROBOT_IP = "192.168.80.3"#os.environ['ROBOT_IP']
SPOT_USERNAME = "admin"#os.environ['SPOT_USERNAME']
SPOT_PASSWORD = "2zqa8dgw7lor"#os.environ['SPOT_PASSWORD']

if __name__ == '__main__':
    with SpotController(username=SPOT_USERNAME, password=SPOT_PASSWORD, robot_ip=ROBOT_IP) as spot:
        spot.image()
    print("done")