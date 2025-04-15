import serial
import time
import numpy as np
from robot import Robot, TrajectoryGenerator

# Set your serial port 
# Please check the port you connec the arduino board before running the code
SERIAL_PORT = "COM3"  # For Windows: "COMx", for Linux/Mac: "/dev/ttyUSB0"
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

    # 1. Go to pickup pose
    pickup_pose = np.eye(4)
    pickup_pose[:3, 3] = np.array([x_start, y_start, pickup_z])
    q_pickup = robot._inverse_kinematics(pickup_pose, seed)
    if q_pickup is None:
        print("Failed IK for pickup.")
        return

    # 2. Lift up
    lift_pose = np.eye(4)
    lift_pose[:3, 3] = np.array([x_start, y_start, pickup_z + lift_height])
    q_lift = robot._inverse_kinematics(lift_pose, q_pickup)
    if q_lift is None:
        print("Failed IK for lift.")
        return

    # 3. Move to above drop-off
    drop_lift_pose = np.eye(4)
    drop_lift_pose[:3, 3] = np.array([x_end, y_end, dropoff_z + lift_height])
    q_drop_lift = robot._inverse_kinematics(drop_lift_pose, q_lift)
    if q_drop_lift is None:
        print("Failed IK for move to drop zone.")
        return

    # 4. Lower down at drop-off
    drop_pose = np.eye(4)
    drop_pose[:3, 3] = np.array([x_end, y_end, dropoff_z])
    q_drop = robot._inverse_kinematics(drop_pose, q_drop_lift)
    if q_drop is None:
        print("Failed IK for final drop.")
        return

    steps = [
        (seed, q_pickup),
        (q_pickup, q_lift),
        (q_lift, q_drop_lift),
        (q_drop_lift, q_drop)
    ]

    for q_start, q_end in steps:
        traj = traj_gen.generate_trapezoidal_trajectory(
            q_start, q_end, traj_gen.max_vel, traj_gen.max_acc, duration=2.0
        )
        traj_gen.follow_joint_trajectory(traj, send_motor_command)

    print("Completed full motion from pickup to dropoff.")

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

# ========== MAIN FUNCTION ==========
def main():
    # Initialize robot & trajectory objects
    robot = Robot()
    traj_gen = TrajectoryGenerator()

    # Define target pose (4x4 transformation matrix)
    target_pose = np.eye(4)
    target_pose[:3, 3] = [0.2, 0.1, 0.2]  # Desired XYZ position of end-effector (example)

    # Initial joint seed (e.g., all zeros)
    seed = np.zeros(robot.dof)

    # Inverse Kinematics
    solution = robot._inverse_kinematics(target_pose, seed)

    if solution is None:
        print("IK failed. Could not reach the desired pose.")
        return

    print("IK solution (radians):", solution)

    # Trajectory generation
    trajectory = traj_gen.generate_trapezoidal_trajectory(seed, solution, traj_gen.max_vel, traj_gen.max_acc, duration=4.0)

    # Send joint angles to Arduino
    print("Sending trajectory to Arduino...")
    traj_gen.follow_joint_trajectory(trajectory, send_motor_command)
    print("Motion complete.")

# ========== RUN MAIN ==========
if __name__ == "__main__":
    main()
    arduino.close()
    print("Connection closed.")