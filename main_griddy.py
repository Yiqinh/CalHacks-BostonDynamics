import os
import time
from spot_controller import SpotController
import math

ROBOT_IP = "192.168.80.3"#os.environ['ROBOT_IP']
SPOT_USERNAME = "admin"#os.environ['SPOT_USERNAME']
SPOT_PASSWORD = "2zqa8dgw7lor"#os.environ['SPOT_PASSWORD']


def capture_image():
#    camera_capture = cv2.VideoCapture(0)
#    rv, image = camera_capture.read()
#    print(f"Image Dimensions: {image.shape}")
#    camera_capture.release()
    pass


def main():
    #example of using micro and speakers
    #print("Start recording audio")
    #sample_name = "aaaa.wav"
    #cmd = f'arecord -vv --format=cd --device={os.environ["AUDIO_INPUT_DEVICE"]} -r 48000 --duration=10 -c 1 {sample_name}'
    #print(cmd)
    #os.system(cmd)
    #print("Playing sound")
    #os.system(f"ffplay -nodisp -autoexit -loglevel quiet {sample_name}")

    # # Capture image

    # Use wrapper in context manager to lease control, turn on E-Stop, power on the robot and stand up at start
    # and to return lease + sit down at the end
    with SpotController(username=SPOT_USERNAME, password=SPOT_PASSWORD, robot_ip=ROBOT_IP) as spot:

        time.sleep(2)
        # Move head to specified positions with intermediate time.sleep
        spot.move_head_in_points(yaws=[0, 0],
                                 pitches=[0.5, -0.5],
                                 rolls=[0, 0],
                                 sleep_after_point_reached=0.3)
        # spot.make_stance(0.5, 0)
        # spot.make_stance(0, 0.5)
        # spot.make_stance(-0.5, 0)
        # spot.make_stance(0, -0.5)
        # spot.make_stance(1, 0)
        # spot.make_stance(0, 1)
        # spot.make_stance(-1, 0)
        # spot.make_stance(0, -1)

        # spot.move_by_velocity_control(0, 0, math.pi / 4, 8)

        perform_griddy(spot)

def perform_griddy(controller):
    # Step 1: Stand up and set a neutral stance
    controller.stand_at_height(body_height=0.0)

    # Step 2: Simulate the skipping footwork using velocity control
    for _ in range(4):
        controller.move_by_velocity_control(v_x=0.4, v_y=0.0, cmd_duration=0.4)  # Forward skip
        controller.move_by_velocity_control(v_x=0.0, v_y=0.4, cmd_duration=0.4)  # Side step

    # Step 3: Add head bobbing with pitch control (up and down movement)
    for _ in range(4):
        controller.move_head_in_points(yaws=[0, 0], pitches=[0.3, -0.3], rolls=[0, 0], sleep_after_point_reached=0.3)  # Head bob

    # Step 4: Add playful head yaw (looking side to side)
    controller.move_head_in_points(yaws=[0.3, -0.3], pitches=[0, 0], rolls=[0, 0], sleep_after_point_reached=0.4)

    # Step 5: End with a stance reset to neutral
    controller.stand_at_height(body_height=0.0)

if __name__ == '__main__':
    main()