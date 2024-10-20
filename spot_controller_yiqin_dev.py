import time
import bosdyn.client
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand  # , blocking_sit
from bosdyn.geometry import EulerZXY
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, \
    GRAV_ALIGNED_BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client import math_helpers
import keyboard
import time

import traceback
import sys

import cv2
import numpy as np

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient

VELOCITY_CMD_DURATION = 0.5


class SpotController:
    def __init__(self, username, password, robot_ip):
        self.username = username
        self.password = password
        self.robot_ip = robot_ip

        sdk = bosdyn.client.create_standard_sdk('ControllingSDK')

        self.robot = sdk.create_robot(robot_ip)
        id_client = self.robot.ensure_client('robot-id')

        self.robot.authenticate(username, password)
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.robot.logger.info("Authenticated")

        self._lease_client = None
        self._lease = None
        self._lease_keepalive = None

        self._estop_client = self.robot.ensure_client(EstopClient.default_service_name)
        self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        self._estop_keepalive = None

        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)

    def image(self, camera):
        #choices=['frontleft', 'frontright', 'left', 'right', 'back', 'hand']
        to_depth = False
        if camera != 'hand':
            sources = [camera + '_depth_in_visual_frame', camera + '_fisheye_image']
        else:
            if to_depth:
                sources = [camera + '_depth', camera + '_color_in_hand_depth_frame']
            else:
                sources = [camera + '_depth_in_hand_color_frame', camera + '_color_image']

        image_responses = self.image_client.get_image_from_sources(sources)
        if len(image_responses) < 2:
                print('Error: failed to get images.')
                return False
        
        cv_depth = np.frombuffer(image_responses[0].shot.image.data, dtype=np.uint16)
        cv_depth = cv_depth.reshape(image_responses[0].shot.image.rows,image_responses[0].shot.image.cols)
        cv_visual = cv2.imdecode(np.frombuffer(image_responses[1].shot.image.data, dtype=np.uint8), -1)
        visual_rgb = cv_visual if len(cv_visual.shape) == 3 else cv2.cvtColor(
        cv_visual, cv2.COLOR_GRAY2RGB)

        # Map depth ranges to color
        # cv2.applyColorMap() only supports 8-bit; convert from 16-bit to 8-bit and do scaling
        min_val = np.min(cv_depth)
        max_val = np.max(cv_depth)
        depth_range = max_val - min_val
        depth8 = (255.0 / depth_range * (cv_depth - min_val)).astype('uint8')
        depth8_rgb = cv2.cvtColor(depth8, cv2.COLOR_GRAY2RGB)
        depth_color = cv2.applyColorMap(depth8_rgb, cv2.COLORMAP_JET)

        # Add the two images together.
        out = cv2.addWeighted(visual_rgb, 0.5, depth_color, 0.5, 0)

        auto_rotate = True
        if auto_rotate:
            if camera[0:5] == 'front':
                out = cv2.rotate(out, cv2.ROTATE_90_CLOCKWISE)
                visual_rgb = cv2.rotate(visual_rgb, cv2.ROTATE_90_CLOCKWISE)
                depth8 = cv2.rotate(depth8, cv2.ROTATE_90_CLOCKWISE)

            elif camera[0:5] == 'right':
                out = cv2.rotate(out, cv2.ROTATE_180)
                visual_rgb = cv2.rotate(visual_rgb, cv2.ROTATE_180)
                depth8 = cv2.rotate(depth8, cv2.ROTATE_180)

        filename = f'/tmp/test.jpg'
        cv2.imwrite(filename, visual_rgb)
        return visual_rgb, depth8

    def follow_head(self):
        center = self.find_person()
        while center != [0, 0]:
            movement = [0, 0] - center #up/down, left/right
            #MOVE THE HEAD BY MOVEMENT
            return
    
    def manual_control(self):
        while True:
            if keyboard.is_pressed('up'):
                self.move_by_velocity_control(v_x=0.1, v_y=0, v_rot=0, cmd_duration=2)
            elif keyboard.is_pressed('down'):
                self.move_by_velocity_control(v_x=-0.1, v_y=0, v_rot=0, cmd_duration=2)
            elif keyboard.is_pressed('left'):
                self.move_by_velocity_control(v_x=0, v_y=0.1, v_rot=0, cmd_duration=2)
            elif keyboard.is_pressed('right'):
                self.move_by_velocity_control(v_x=0, v_y=-0.1, v_rot=0, cmd_duration=2)
            elif keyboard.is_pressed('q'):
                print("Quitting control.")
                break
            # Small delay to avoid excessive commands
            time.sleep(0.1)

    def find_person(self):
        #find a person's face
        return [0, 0]

    def release_estop(self):
        self._estop_endpoint.force_simple_setup()
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def set_estop(self):
        if self._estop_keepalive:
            try:
                self._estop_keepalive.stop()
            except:
                self.robot.logger.error("Failed to set estop")
                traceback.print_exc()
            self._estop_keepalive.shutdown()
            self._estop_keepalive = None

    def lease_control(self):
        self._lease_client = self.robot.ensure_client('lease')
        self._lease = self._lease_client.take()
        self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client, must_acquire=True)
        self.robot.logger.info("Lease acquired")

    def return_lease(self):
        self._lease_client.return_lease(self._lease)
        self._lease_keepalive.shutdown()
        self._lease_keepalive = None

    def __enter__(self):
        self.lease_control()
        self.release_estop()
        self.power_on_stand_up()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type:
            self.robot.logger.error("Spot powered off with " + exc_val + " exception")
        self.power_off_sit_down()
        self.return_lease()
        self.set_estop()

        return True if exc_type else False

    def move_head_in_points(self, yaws, pitches, rolls, body_height=0, sleep_after_point_reached=0, timeout=3):
        for i in range(len(yaws)):
            footprint_r_body = EulerZXY(yaw=yaws[i], roll=rolls[i], pitch=pitches[i])
            params = RobotCommandBuilder.mobility_params(footprint_R_body=footprint_r_body, body_height=body_height)
            blocking_stand(self.command_client, timeout_sec=timeout, update_frequency=0.02, params=params)
            self.robot.logger.info("Moved to yaw={} rolls={} pitch={}".format(yaws[i], rolls[i], pitches[i]))
            if sleep_after_point_reached:
                time.sleep(sleep_after_point_reached)

    def wait_until_action_complete(self, cmd_id, timeout=15):
        start_time = time.time()
        while time.time() - start_time < timeout:
            feedback = self.command_client.robot_command_feedback(cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                print("Failed to reach the goal")
                return False
            traj_feedback = mobility_feedback.se2_trajectory_feedback
            if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                    traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
                print("Arrived at the goal.")
                return True
            time.sleep(0.5)

    def move_to_goal(self, goal_x=0, goal_y=0):
        cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
            goal_x_rt_body=goal_x,
            goal_y_rt_body=goal_y,
            goal_heading_rt_body=0,
            frame_tree_snapshot=self.robot.get_frame_tree_snapshot()
        )
        # cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=goal_x, goal_y=goal_y, goal_heading=0,
        #                                                                frame_name=GRAV_ALIGNED_BODY_FRAME_NAME)
        cmd_id = self.command_client.robot_command(lease=None, command=cmd,
                                                   end_time_secs=time.time() + 10)
        self.wait_until_action_complete(cmd_id)

        self.robot.logger.info("Moved to x={} y={}".format(goal_x, goal_y))

    def power_on_stand_up(self):
        self.robot.power_on(timeout_sec=20)
        assert self.robot.is_powered_on(), "Not powered on"
        self.robot.time_sync.wait_for_sync()
        blocking_stand(self.command_client, timeout_sec=10)

    def power_off_sit_down(self):
        self.move_head_in_points(yaws=[0], pitches=[0], rolls=[0])
        self.robot.power_off(cut_immediately=False)

    def make_stance(self, x_offset, y_offset):
        state = self.state_client.get_robot_state()
        vo_T_body = get_se2_a_tform_b(state.kinematic_state.transforms_snapshot,
                                      VISION_FRAME_NAME,
                                      GRAV_ALIGNED_BODY_FRAME_NAME)

        pos_fl_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, y_offset, 0)
        pos_fr_rt_vision = vo_T_body * math_helpers.SE2Pose(x_offset, -y_offset, 0)
        pos_hl_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, y_offset, 0)
        pos_hr_rt_vision = vo_T_body * math_helpers.SE2Pose(-x_offset, -y_offset, 0)

        stance_cmd = RobotCommandBuilder.stance_command(
            VISION_FRAME_NAME, pos_fl_rt_vision.position,
            pos_fr_rt_vision.position, pos_hl_rt_vision.position, pos_hr_rt_vision.position)

        start_time = time.time()
        while time.time() - start_time < 6:
            # Update end time
            stance_cmd.synchronized_command.mobility_command.stance_request.end_time.CopyFrom(
                self.robot.time_sync.robot_timestamp_from_local_secs(time.time() + 5))
            # Send the command
            self.command_client.robot_command(stance_cmd)
            time.sleep(0.1)

    def move_by_velocity_control(self, v_x=0.0, v_y=0.0, v_rot=0.0, cmd_duration=VELOCITY_CMD_DURATION):
        # v_x+ - forward, v_y+ - left | m/s, v_rot+ - counterclockwise |rad/s
        self._start_robot_command(
            RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + cmd_duration)

    def _start_robot_command(self, command_proto, end_time_secs=None):
        self.command_client.robot_command(lease=None, command=command_proto, end_time_secs=end_time_secs)

    def stand_at_height(self, body_height):
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=body_height)
        self.command_client.robot_command(cmd)

    def bow(self, pitch, body_height=0, sleep_after_point_reached=0):
        self.move_head_in_points([0, 0], [pitch, 0], [0, 0], body_height=body_height,
                                 sleep_after_point_reached=sleep_after_point_reached, timeout=3)

    def dust_off(self, yaws, pitches, rolls):
        self.move_head_in_points(yaws, pitches, rolls, sleep_after_point_reached=0, body_height=0)
