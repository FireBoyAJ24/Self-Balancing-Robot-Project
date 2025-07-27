import serial
import time
import matplotlib.pyplot as plt

def set_pid(kp, ki, kd):
    ser.write(f'P={kp}\n'.encode())
    ser.write(f'I={ki}\n'.encode())
    ser.write(f'D={kd}\n'.encode())

arduino_port = 'COM5'     
baud_rate = 115200

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset and start sending

    print("Reading data from Arduino. Press Ctrl+C to stop.")
    set_pid(1.0, 0.0, 0.0)
    
    data = []
    start_time = time.time()
    duration = 20  # seconds

    while time.time() - start_time < duration:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                value = float(line)
                data.append(value)
                # print(f"Received: {value}")
            except ValueError:
                print(f"Non-numeric data received: {line}")

    ser.close()

    plt.plot(data)
    plt.title("Arduino Serial Data")
    plt.xlabel("Sample")
    plt.ylabel("Pitch (degrees)")
    plt.show()

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    ser.close()
    print("\nStopped by user.")
