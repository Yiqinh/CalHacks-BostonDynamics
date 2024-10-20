import time
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_state import RobotStateClient

# Connect to Spot
def connect_to_robot(robot_ip):
    sdk = bosdyn.client.create_standard_sdk('GriddySpotClient')
    robot = sdk.create_robot(robot_ip)
    robot.authenticate('username', 'password')
    robot.time_sync.wait_for_sync()
    return robot

# Make Spot perform a "Griddy" dance
def perform_griddy(robot):
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    lease_client = robot.ensure_client(LeaseClient.default_service_name)

    # Take control of Spot using a lease
    with lease_client.take():
        # Step 1: Stand up
        blocking_stand(command_client)
        
        # Step 2: Simulate skipping footwork (alternating leg lifts with forward movement)
        for _ in range(4):  # Repeat to create the skipping motion
            lift_leg(command_client, leg_id=0, duration=0.4)  # Lift front left leg
            move_forward(command_client, duration=0.4)
            lift_leg(command_client, leg_id=1, duration=0.4)  # Lift front right leg
            move_forward(command_client, duration=0.4)

        # Step 3: Add head bobbing (up and down movement)
        for _ in range(4):
            move_head(command_client, pitch=0.3, duration=0.3)  # Tilt head down
            move_head(command_client, pitch=-0.3, duration=0.3)  # Tilt head up

        # Step 4: Add playful head yaw (looking side to side)
        move_head(command_client, yaw=0.5, duration=0.4)  # Look left
        move_head(command_client, yaw=-0.5, duration=0.4)  # Look right

        # Step 5: Return to a neutral standing position
        blocking_stand(command_client)

# Function to lift a leg
def lift_leg(command_client, leg_id, duration):
    # Joint positions for lifting legs
    if leg_id == 0:  # Front left leg
        joint_positions = [
            RobotCommandBuilder.build_joint_position('fl.hx', 0.5),  # Hip X
            RobotCommandBuilder.build_joint_position('fl.hy', 0.2),  # Hip Y
            RobotCommandBuilder.build_joint_position('fl.kn', -0.5)  # Knee
        ]
    elif leg_id == 1:  # Front right leg
        joint_positions = [
            RobotCommandBuilder.build_joint_position('fr.hx', 0.5),  # Hip X
            RobotCommandBuilder.build_joint_position('fr.hy', -0.2),  # Hip Y
            RobotCommandBuilder.build_joint_position('fr.kn', -0.5)  # Knee
        ]

    command = RobotCommandBuilder.synchro_stand_command(joint_positions)
    command_client.robot_command(command)
    time.sleep(duration)  # Hold the leg in the air for the specified duration

# Function to move Spot forward
def move_forward(command_client, duration):
    # Command to move forward in a straight line
    velocity_cmd = RobotCommandBuilder.se2_velocity_command(v_x=0.5, v_y=0, v_rot=0)
    command_client.robot_command(velocity_cmd)
    time.sleep(duration)

# Function to move Spot's head
def move_head(command_client, pitch=0.0, yaw=0.0, duration=0.4):
    # Adjust head orientation (pitch and yaw)
    head_orientation = RobotCommandBuilder.build_head_tilt_command(pitch=pitch, yaw=yaw)
    command_client.robot_command(head_orientation)
    time.sleep(duration)

# Helper function to make Spot stand in neutral position
def blocking_stand(command_client):
    stand_command = RobotCommandBuilder.synchro_stand_command()
    command_client.robot_command(stand_command)
    time.sleep(1)  # Allow Spot to adjust

if __name__ == "__main__":
    robot_ip = "192.168.50.3"  # Replace with Spot's IP
    robot = connect_to_robot(robot_ip)
    perform_griddy(robot)