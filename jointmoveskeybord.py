import numpy as np
import time
from pyfirmata import ArduinoMega,Arduino, SERVO, util, INPUT
import curses

import serial
import time
import numpy as np
'''
# Replace with the correct port for your ESP32
ESP32_PORT = "/dev/ttyUSB0"  # Linux/macOS (use "COMx" on Windows)
BAUD_RATE = 115200


esp = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for ESP32 to initialize

def servoMove(pin, angle):
    command = f"servoMove,{pin},{angle}".strip().upper()
    esp.write((command + "\n").encode())  # Send command
    print(a,b)
a = 1.1
b= 0.34
while True:
    a+=b
    b+=b
    servoMove(a,b)
    time.sleep(1)
'''


# Define servo positions and offsets
#expected = np.array([[0], [90], [-90], [0], [0], [0]])
#home = np.array([[90], [90], [30], [90], [90], [90]])
#homerad = home * ((2 * np.pi) / 180)

home = [0]
# Initialize the Arduino board
board = Arduino('/dev/ttyACM0')

# Define and activate servos
servo_pins = [9]

print('Sleeping for 1 second...')
time.sleep(1)

def control_servos_with_keyboard():
    # Initialize curses screen
    screen = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen.keypad(True)

    # Define joint positions
    current_positions = [90.]
    for i in range(len(servo_pins)):
                board.digital[servo_pins[i]].mode = SERVO                  #Initilize all joint servos 
                board.digital[servo_pins[i]].write(current_positions[i])           #Go to joint home pos 
                time.sleep(0.001)
    try:
        screen.addstr(0, 0, "Use keys 1-6 to select a servo. Use Up/Down arrows to move. Press 'q' to quit.")

        selected_servo = 0
        while True:
            # Display current servo and positions
            screen.addstr(2, 0, f"Selected Servo: {selected_servo + 1}")
            screen.addstr(3, 0, f"Positions: {current_positions}")
            screen.refresh()

            # Get user input
            key = screen.getch()
            
            if key == ord('q'):  # Quit
                break

            elif key in [ord(str(i + 1)) for i in range(len(servo_pins))]:  # Select servo
                selected_servo = int(chr(key)) - 1
            
            elif key == curses.KEY_UP:  # Increase angle
                if current_positions[selected_servo] < 180:
                    current_positions[selected_servo] += .5
                    board.digital[servo_pins[selected_servo]].write(current_positions[selected_servo])

            elif key == curses.KEY_DOWN:  # Decrease angle
                if current_positions[selected_servo] > 0:
                    current_positions[selected_servo] -= .5
                    board.digital[servo_pins[selected_servo]].write(current_positions[selected_servo])

    except Exception as e:
        print("Error:", e)

    finally:
        # Cleanup curses
        curses.nocbreak()
        screen.keypad(False)
        curses.echo()
        curses.endwin()

# Call the function to start controlling servos
control_servos_with_keyboard()