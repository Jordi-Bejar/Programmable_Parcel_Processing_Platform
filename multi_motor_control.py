import serial
import time
import numpy as np
from robot import Robot, TrajectoryGenerator
import utils 

# Set your serial port 
# Please check the port you connec the arduino board before running the code
SERIAL_PORT = "/dev/tty.usbmodem1101"  # For Windows: "COMx", for Linux/Mac: "/dev/ttyUSB0"
BAUD_RATE = 9600  # Must match the Arduino baud rate

# Open serial connection
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Allow time for Arduino to reset
    print("Connected to Arduino!")
except Exception as e:
    print(f"Error: {e}")
    exit()

# This is the function to move the motor.
# motor_num: 1-5
# angle: positive = clockwise negative = counterclockwise
# speed: scale 0-1
def send_motor_command(motor_num, angle, speed):
    """ Sends a formatted command to the Arduino """
    command = f"{motor_num} {angle} {speed}\n"
    arduino.write(command.encode())  # Send command
    print(f"Sent: {command.strip()}")
    
    # Wait for response from Arduino
    response = arduino.readline().decode().strip()
    if response:
        print(f"Arduino: {response}")

def move_from_to(x_start, y_start, x_end, y_end, robot, traj_gen, send_motor_command):
    lift_height = 0.05    
    pickup_z = 0.10        
    dropoff_z = 0.10       
    seed = np.zeros(robot.dof)
    R = np.eye(3)

    current_joint = seed

    # 1. Go to pickup pose
    pickup = [x_start, y_start, pickup_z]
    pickup_above = [x_start, y_start, pickup_z + lift_height]
    drop_above = [x_end, y_end, dropoff_z + lift_height]
    drop = [x_end, y_end, dropoff_z]

    steps = [
        (current_joint, pickup_above, pickup),      # move down
        (None, pickup, pickup_above),               # lift up
        (None, pickup_above, drop_above),           # move across
        (None, drop_above, drop),                   # move down
    ]

    for i, (q_guess, pt1, pt2) in enumerate(steps):
        if q_guess is None:
            q_guess = current_joint
        traj = traj_gen.generate_straight_line(pt1, pt2, q_guess, R, duration=2)
        traj_gen.follow_joint_trajectory(traj, send_motor_command)
        current_joint = traj[-1]

    print("Completed full straight-line motion from pickup to dropoff.")

# Example commands

#send_motor_command(1, 180, 1.0)
#send_motor_command(1, 90, 0.5)  # Move Motor 1, 90 degrees at 50% speed
#time.sleep(1)
#send_motor_command(2, 180, 1.0)  # Move Motor 2, 180 degrees at full speed
#time.sleep(1)
#send_motor_command(3, 360, 0.7)  # Move Motor 3, 45 degrees at 20% speed
#time.sleep(1)
#send_motor_command(4, 360, 0.5)  # Move Motor 4, full rotation at 80% speed
#time.sleep(1)
#send_motor_command(5, 360, 0.5)
#send_motor_command(2, 3600, 1.0)
#send_motor_command(3, 3600, 1.0)
#time.sleep(2)
#send_motor_command(1, 90, 1.0)
#send_motor_command(4, -90, 1.0)
#time.sleep(4)
#send_motor_command(1, 3510, 1.0)
#send_motor_command(4, 36090, 1.0)
#time.sleep(4)
#send_motor_command(5, -90, 1.0)
#time.sleep(2)
#send_motor_command(5, 36090, 1.0)

#move_from_to(
#     x_start=0.15,
#     y_start=0.05,
#     x_end=0.25,
#     y_end=-0.05,
#     robot=robot,
#     traj_gen=traj_gen,
#     send_motor_command=send_motor_command
# )

# Close serial connection
#arduino.close()
#print("Connection closed.")

def accumErrorTest():
    robot = Robot()
    traj_gen = TrajectoryGenerator()
    seed = np.zeros(robot.dof)
    R = np.eye(3)

    start_pos = [0.4, 0, 0.3]
    end_pos = [0.2, 0, 0.6]

    # Compute and print IK solutions for start and end
    pose_start = robot._get_transform(R, start_pos)
    pose_end = robot._get_transform(R, end_pos)

    ik_start = robot._inverse_kinematics(pose_start, seed)
    ik_end = robot._inverse_kinematics(pose_end, seed)

    print("=== IK Solutions ===")
    print("Start Pose:", start_pos)
    print("Start Joint Solution:", ik_start)
    print("End Pose:", end_pos)
    print("End Joint Solution:", ik_end)

    if ik_start is None or ik_end is None:
        print("IK failed for one of the poses.")
        return

    # Move forward
    traj_forward = traj_gen.generate_straight_line(start_pos, end_pos, ik_start, R, duration=3)
    traj_gen.follow_joint_trajectory(traj_forward, send_motor_command)
    current_joint = traj_forward[-1]

    while True:
        input("Press enter to go backward...")
        traj_backward = traj_gen.generate_straight_line(end_pos, start_pos, current_joint, R, duration=3)
        traj_gen.follow_joint_trajectory(traj_backward, send_motor_command)
        current_joint = traj_backward[-1]

        print("Returning to original pose. Joint state after reverse:")
        print(current_joint)
#def itemFragileTest():



    # Trajectory generation
    # trajectory = traj_gen.generate_trapezoidal_trajectory(seed, solution, traj_gen.max_vel, traj_gen.max_acc, duration=4.0)
    # traj_gen.follow_joint_trajectory(trajectory, send_motor_command)  

    # while :


def weightVerTest():
    robot = Robot()
    traj_gen = TrajectoryGenerator()

    target_pose1 = [0, 3.14159/4, 0, 0, 0]
    target_pose2 = [0, 0, 3.14159/2, 0, 0]
    target_pose3 = [0, 0, 0, 3.14159/2, 0]

    # Initial joint seed (e.g., all zeros)
    seed = np.zeros(robot.dof)

    trajectory = traj_gen.generate_trapezoidal_trajectory(seed, target_pose1, traj_gen.max_vel, traj_gen.max_acc, duration=2.0)
    traj_gen.follow_joint_trajectory(trajectory, send_motor_command)
    print("Joint 2 test")
    input()

    trajectory = traj_gen.generate_trapezoidal_trajectory(target_pose1, target_pose2, traj_gen.max_vel, traj_gen.max_acc, duration=2.0)
    traj_gen.follow_joint_trajectory(trajectory, send_motor_command)
    print("Joint 3 test")
    input()

    trajectory = traj_gen.generate_trapezoidal_trajectory(target_pose2, target_pose3, traj_gen.max_vel, traj_gen.max_acc, duration=2.0)
    traj_gen.follow_joint_trajectory(trajectory, send_motor_command)
    print("Joint 4 test")
    input()
    

def pickPlaceTest():
    target_pose1 = np.eye(4)
    target_pose1[:3, 3] = [0.3, 0, 0.1]  # Desired XYZ position of end-effector (example)

    trajectory = traj_gen.generate_trapezoidal_trajectory(seed, target_pose1, traj_gen.max_vel, traj_gen.max_acc, duration=2.0)
    traj_gen.follow_joint_trajectory(trajectory, send_motor_command)

    while True:

        move_from_to(
            x_start=0.3,
            y_start=0,
            x_end=0.6,
            y_end=0,
            robot=robot,
            traj_gen=traj_gen,
            send_motor_command=send_motor_command
        )
        print("(waiting)")
        input()
        print("(moving)")

        move_from_to(
            x_start=0.6,
            y_start=0,
            x_end=0.3,
            y_end=0,
            robot=robot,
            traj_gen=traj_gen,
            send_motor_command=send_motor_command
        )

        print("(waiting)")
        input()
        print("(moving)")


# ========== MAIN FUNCTION ==========
def main():
    accumErrorTest()
    # Initialize robot & trajectory objects
    

# ========== RUN MAIN ==========
if __name__ == "__main__":
    main()
    arduino.close()
    print("Connection closed.")