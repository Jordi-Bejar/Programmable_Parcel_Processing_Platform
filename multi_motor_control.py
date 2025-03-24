import serial
import time

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
send_motor_command(2, 3600, 1.0)
send_motor_command(3, 3600, 1.0)
time.sleep(2)
send_motor_command(1, 90, 1.0)
send_motor_command(4, -90, 1.0)
time.sleep(4)
send_motor_command(1, 3510, 1.0)
send_motor_command(4, 36090, 1.0)
time.sleep(4)
send_motor_command(5, -90, 1.0)
time.sleep(2)
send_motor_command(5, 36090, 1.0)

# Close serial connection
arduino.close()
print("Connection closed.")
