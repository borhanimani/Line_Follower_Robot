import cv2
import serial
import time
import numpy as np
import threading

# Serial settings (adjust COM port and baud rate)
SERIAL_PORT = 'COM11'
BAUD_RATE = 78600

# Robot physical parameters (in pixels)
ROBOT_LENGTH = 60  # 15 cm scaled to 60 pixels
ROBOT_WIDTH = 28   # 7 cm scaled to 28 pixels
WHEEL_BASE = 30    # distance between rear wheels in pixels
ROBOT_RADIUS = 10
MAX_PWM = 255
PIXELS_PER_TICK = 1.0  # encoder resolution

# Sensor offsets (placed at front edge, spaced across width)
SENSOR_FRONT_OFFSET = ROBOT_LENGTH // 2  # from center to front
SENSOR_POSITIONS = [
    (SENSOR_FRONT_OFFSET, -ROBOT_WIDTH // 2 + i * (ROBOT_WIDTH // 4) )
    for i in range(5)
]  # 5 sensors left to right

# Load map image (black line on white background)
MAP = cv2.imread('track.bmp', cv2.IMREAD_GRAYSCALE)
MAP_COLOR = cv2.cvtColor(MAP, cv2.COLOR_GRAY2BGR)



# Initial robot state
x, y, angle = 100, 100, 0  # pixels, radians
left_encoder = 0
right_encoder = 0

# Lock for thread safety
lock = threading.Lock()

# Serial communication thread
def serial_thread():
    global x, y, angle, left_encoder, right_encoder
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("[Serial] Ready")

    while True:
        try:
            line = ser.readline().decode().strip()
            if line.startswith("L:"):
                parts = line.split(',')
                left_pwm = int(parts[0].split(':')[1])
                right_pwm = int(parts[1].split(':')[1])

                # Normalize motor speeds
                vl = (left_pwm - 128) / 128.0  # range -1 to +1
                vr = (right_pwm - 128) / 128.0

                # Calculate motion
                dt = 0.1  # timestep in seconds
                v = (vr + vl) / 2 * 50  # speed scaling
                w = (vr - vl) / WHEEL_BASE * 50

                with lock:
                    # Update pose
                    angle += w * dt
                    x += v * np.cos(angle) * dt
                    y += v * np.sin(angle) * dt

                    # Encoder tick simulation
                    left_encoder += int(abs(vl * dt * 50 / PIXELS_PER_TICK))
                    right_encoder += int(abs(vr * dt * 50 / PIXELS_PER_TICK))

                    # Sensor detection (5 sensors at front)
                    sensor_vals = []
                    for dx, dy in SENSOR_POSITIONS:
                        sx = int(x + dx * np.cos(angle) - dy * np.sin(angle))
                        sy = int(y + dx * np.sin(angle) + dy * np.cos(angle))
                        if 0 <= sx < MAP.shape[1] and 0 <= sy < MAP.shape[0]:
                            pixel = MAP[sy, sx]
                            val = 1 if pixel < 100 else 0
                        else:
                            val = 0
                        sensor_vals.append(val)

                # Send sensor and encoder data
                sensor_str = ','.join(str(s) for s in sensor_vals)
                enc_str = f"{left_encoder},{right_encoder}"
                ser.write(f"S:{sensor_str};E:{enc_str}\n".encode())

        except Exception as e:
            print("Serial error:", e)

# Start serial communication thread
threading.Thread(target=serial_thread, daemon=True).start()

window_name = "Line Follower Simulation"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
# GUI loop
while True:
    with lock:
        frame = MAP_COLOR.copy()
        # Draw robot as a red circle
        cv2.circle(frame, (int(x), int(y)), ROBOT_RADIUS, (0, 0, 255), -1)
        # Draw sensors
        for dx, dy in SENSOR_POSITIONS:
            sx = int(x + dx * np.cos(angle) - dy * np.sin(angle))
            sy = int(y + dx * np.sin(angle) + dy * np.cos(angle))
            cv2.circle(frame, (sx, sy), 3, (255, 255, 0), -1)

    cv2.imshow(window_name, frame)
    key = cv2.waitKey(50)
    if key == 27:
        break
    if key == 48:
        x, y, angle = 100, 100, 0 

cv2.destroyAllWindows()
