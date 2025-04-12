import serial
import time
import numpy as np

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