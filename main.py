import os
import time
from spot_controller import SpotController
import numpy as np
import cv2 as cv
import math

ROBOT_IP = "192.168.80.3"#os.environ['ROBOT_IP']
SPOT_USERNAME = "admin"#os.environ['SPOT_USERNAME']
SPOT_PASSWORD = "2zqa8dgw7lor"#os.environ['SPOT_PASSWORD']


def capture_image():
    camera_capture = cv.VideoCapture(0)
    rv, image = camera_capture.read()
    print(f"Image Dimensions: {image.shape}")
    camera_capture.release()
    return image


def main():
    #example of using micro and speakers
    print("Start recording audio")
    sample_name = "aaaa.wav"
    cmd = f'arecord -vv --format=cd --device={os.environ["AUDIO_INPUT_DEVICE"]} -r 48000 --duration=10 -c 1 {sample_name}'
    print(cmd)
    os.system(cmd)
    print("Playing sound")
    os.system(f"ffplay -nodisp -autoexit -loglevel quiet {sample_name}")

    # # Capture image

    # Use wrapper in context manager to lease control, turn on E-Stop, power on the robot and stand up at start
    # and to return lease + sit down at the end
    with SpotController(username=SPOT_USERNAME, password=SPOT_PASSWORD, robot_ip=ROBOT_IP) as spot:
        while True:
            img = capture_image()
            rects = detect(img)
            avg_pos = (0,0)
            for rect in rects:
                avg_pos[0] += rect[0] + rect[2]/2
                avg_pos[1] += rect[1] + rect[3]/2

            print("AVG POS")
            print(avg_pos)

            yaw = 0
            pitch = 0
            if avg_pos[0] < img.shape[0] / 2:
                yaw = -0.1
            else:
                yaw = 0.1
            if avg_pos[1] < img.shape[1] / 2:
                pitch = -0.1
            else:
                pitch = 0.1
            
            print("YAW PITCH")
            print(yaw, pitch)
            
            spot.move_head_in_points(yaws=[yaw],pitches=[pitch],rolls=[0],sleep_after_point_reached=0.1)

def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh


def draw_detections(img, rects, thickness = 1):
    for x, y, w, h in rects:
        # the HOG detector returns slightly larger rectangles than the real objects.
        # so we slightly shrink the rectangles to get a nicer output.
        pad_w, pad_h = int(0.15*w), int(0.05*h)
        cv.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0, 255, 0), thickness)


def detect(image):
    import sys
    from glob import glob
    import itertools as it

    hog = cv.HOGDescriptor()
    hog.setSVMDetector( cv.HOGDescriptor_getDefaultPeopleDetector() )

    default = [image]

    for img in default:
        found, _w = hog.detectMultiScale(img, winStride=(8,8), padding=(32,32), scale=1.05)
        found_filtered = []
        for ri, r in enumerate(found):
            for qi, q in enumerate(found):
                if ri != qi and inside(r, q):
                    break
            else:
                found_filtered.append(r)
        print(found)
        print('%d (%d) found' % (len(found_filtered), len(found)))

    print('Done')

if __name__ == '__main__':
    main()
