import serial
import time

# Replace with your ESP32 serial port
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to initialize
    print(f"Serial port {SERIAL_PORT} opened successfully!")
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

# Send data every second
while True:
    msg = "Blink\n"  # Any string, just needs to end with newline
    ser.write(msg.encode())
    print("Sent:", msg.strip())
    time.sleep(1)
