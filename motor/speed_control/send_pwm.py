import serial
import time

# Replace with your ESP32 serial port
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to reset
    print(f"Serial port {SERIAL_PORT} opened successfully!")
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

while True:
    try:
        # Ask user for PWM input
        pwm_input = input("Enter PWM (0-255) or PWM1,PWM2: ")
        pwm_input = pwm_input.strip()
        
        # Send to ESP32
        ser.write((pwm_input + '\n').encode())
        print("Sent:", pwm_input)
        
        # Read ESP32 response
        time.sleep(0.1)
        while ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print("ESP32:", response)
    except KeyboardInterrupt:
        print("Exiting...")
        ser.close()
        break
