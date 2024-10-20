import os
import time
from spot_controller import SpotController
import math

ROBOT_IP = "192.168.80.3"  # os.environ['ROBOT_IP']
SPOT_USERNAME = "admin"  # os.environ['SPOT_USERNAME']
SPOT_PASSWORD = "2zqa8dgw7lor"  # os.environ['SPOT_PASSWORD']


def main():
    # Use SpotController to lease control, power on the robot, and stand up at start
    with SpotController(username=SPOT_USERNAME, password=SPOT_PASSWORD, robot_ip=ROBOT_IP) as spot:

        perform_griddy(spot)
        # look_up_walk(spot)

def look_up_walk(controller):
    controller.stand_at_height(body_height=0.0)
    time.sleep(1)


def perform_griddy(controller):
    # Step 1: Stand up and set neutral stance
    controller.stand_at_height(body_height=0.0)
    time.sleep(1)

    controller.make_stance(x_offset=0.0, y_offset=0.05)
    time.sleep(2)
    controller.make_stance(x_offset=0.0, y_offset=-0.05)
    time.sleep(2)

    # Step 2: Simulate skipping footwork using velocity control and leg lifting
    # for _ in range(4):
    #     # Forward skip
    #     controller.move_by_velocity_control(v_x=0.4, v_y=0.0, cmd_duration=1.5)
    #     time.sleep(0.3)

    #     # Side step
    #     controller.move_by_velocity_control(v_x=0.0, v_y=0.4, cmd_duration=0.5)
    #     time.sleep(0.3)

    #     # Lift front-left leg, then back-right leg to simulate skipping
    #     # controller.make_stance(x_offset=0.2, y_offset=0.1)  # Lift front-left leg
    #     # time.sleep(0.3)
    #     # controller.make_stance(x_offset=-0.2, y_offset=-0.1)  # Lift back-right leg
    #     # time.sleep(0.3)

    # # Step 3: Head bobbing with pitch control (up and down movement)
    # for _ in range(2):
    #     controller.move_head_in_points(yaws=[0, 0], pitches=[0.3, -0.3], rolls=[0, 0], sleep_after_point_reached=0.3)
    #     time.sleep(0.3)

    # # Step 4: Add playful head yaw (looking side to side)
    # controller.move_head_in_points(yaws=[0.3, -0.3], pitches=[0, 0], rolls=[0, 0], sleep_after_point_reached=0.4)

    # Step 5: End with a stance reset to neutral
    controller.stand_at_height(body_height=0.0)


if __name__ == '__main__':
    main()