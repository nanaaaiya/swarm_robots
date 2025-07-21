import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Wait for serial connection to establish

def send_velocities(left_vel, right_vel):
    # Format the velocities as "left,right\n"
    command = f"{left_vel},{right_vel}\n"
    ser.write(command.encode())  # Send the command
    time.sleep(0.1)  # Small delay to ensure data is sent
    
    # Read and print the response from Arduino
    if ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        print(response)

def main():
    try:
        while True:
            print("Sending velocities: 0.3, 0.3")
            send_velocities(0.3, 0.3)
            time.sleep(2)  # Wait for 2 seconds
            
            print("Sending velocities: -0.3, -0.3")
            send_velocities(-0.3, -0.3)
            time.sleep(2)
            
            print("Sending velocities: 0, 0")
            send_velocities(0, 0)
            time.sleep(2)
            
    except KeyboardInterrupt:
        send_velocities(0, 0)
        print("\nProgram terminated by user")
    finally:
        ser.close()  # Close the serial connection

if __name__ == "__main__":
    main()