# =============================================================================
# Example Car Navigator File
# -----------------------------------------------------------------------------
# This file is intended as a reference example. Competitors or developers can:
#   - Use it as a baseline for their own navigation systems
#   - Modify and extend it for improved performance or new features
#   - Integrate it into larger autonomous driving pipelines
#
# This script demonstrates a complete baseline implementation of a car
# navigation and control system using path planning, communication streams,
# and a Stanley controller for steering.
#
# The implementation includes:
#   - Path following via Stanley controller
#   - TCP/IP communication with simulation/client
#   - Optional camera streaming
#   - Keyboard-based mode switching
#
# The QCar2 Limits:
#   - The velocity command must be in the interval [-0.2, 0.2]
#   - The steering command must be in the interval  [-0.6, 0.6]
# =============================================================================


# region: Python level imports
# Standard libraries for threading and inter-thread communication
import threading
import queue

# Numerical and computer vision libraries
import numpy as np
import cv2

# File path handling
from pathlib import Path

# Quanser-specific communication timeout handling
try:
    from quanser.common import Timeout
except:
    from quanser.communications import Timeout

# Quanser platform utilities for streaming, timing, and cameras
from pal.utilities.stream import BasicStream
from pal.utilities.timing import QTimer
from pal.utilities.vision import Camera2D
# endregion 


# =============================================================================
# Utility Functions
# =============================================================================

def read_initial_positions(filepath: Path) -> np.ndarray:
    """
    Reads initial spawn positions from a text file.

    The file is expected to contain comma-separated numeric values.
    Lines starting with '#' are treated as comments.

    Returns:
        np.ndarray: First 4 numeric values representing initial pose data.
    """
    if not filepath.exists():
        raise FileNotFoundError(f"Spawn file not found: {filepath}")

    values: list[float] = []

    with filepath.open("r", encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            line = raw_line.strip()

            # Skip empty lines and comments
            if not line or line.startswith("#"):
                continue

            try:
                parts = [float(value.strip()) for value in line.split(",")]
            except ValueError as exc:
                raise ValueError(
                    f"Invalid numeric value in {filepath} on line {line_number}: {raw_line.strip()}"
                ) from exc

            values.extend(parts)

    # Ensure minimum required values exist
    if len(values) < 8:
        raise ValueError(
            f"{filepath} must contain at least 8 numeric values, but found {len(values)}."
        )

    return np.array(values[0:4], dtype=np.float64)


def keyboard_listener(mode_queue: queue.Queue):
    """
    Runs in a separate thread to listen for user keyboard input.

    Allows runtime switching between predefined driving modes.
    Input values:
        task1 -> Stop / idle
        task2 -> Navigate to pickup location    
    """
    print("Keyboard control is active.")
    print("Initial mode is task1.")
    print("Type task1 to stop.")
    print("Type task2 to go to Central Pickup Location and pick up two small packages.")
    

    while True:
        try:
            user_input = input().strip()
        except EOFError:
            break

        if user_input in ("task1", "task2"):
            mode_queue.put(user_input)
        else:
            print("Invalid input. Enter only task1 or task2.")


def load_plan_file(plan_path: Path):
    """
    Loads precomputed path planning data from a NumPy file.

    Returns:
        ndarray: Path poses used for navigation
    """
    qcar2_pathposes = np.load(plan_path, allow_pickle=True)
    return qcar2_pathposes


def stanley_controller(pose, vel_cmd, path_pose):
    """
    Stanley Controller Implementation

    Computes steering and velocity adjustments to follow a reference path.

    Core idea:
        - Minimize heading error (orientation mismatch)
        - Minimize cross-track error (lateral deviation)

    Returns:
        vel_cmd: Updated velocity command
        delta: Steering angle command
    """

    # ---------------- Controller Parameters ----------------
    epsilon = 1e-3   # Prevent division by zero
    k = 2.5          # Control gain
    delta_max = 0.6  # Steering saturation limit

    pose = np.asarray(pose, dtype=float)
    path_pose = np.asarray(path_pose, dtype=float)

    x, y, yaw = pose[0], pose[1], pose[2]
    path_x = path_pose[:, 0]
    path_y = path_pose[:, 1]
    path_yaw = path_pose[:, 2]

    # ---------------- Steering Control ----------------

    # Compute distance to all path points
    dx = path_x - x
    dy = path_y - y
    d = np.hypot(dx, dy)

    # Select closest path point
    target_idx = np.argmin(d)

    x_ref = path_x[target_idx]
    y_ref = path_y[target_idx]
    yaw_ref = path_yaw[target_idx]

    # Heading error (wrapped to [-pi, pi])
    psi_e = (yaw_ref - yaw + np.pi) % (2 * np.pi) - np.pi

    # Cross-track error (signed lateral distance)
    dxl = x - x_ref
    dyl = y - y_ref
    e_ct = -np.sin(yaw_ref) * dxl + np.cos(yaw_ref) * dyl
    e_ct = -e_ct

    # Stanley control law
    vel = vel_cmd * 13 / 0.2
    delta = psi_e + np.arctan2(k * e_ct, vel + epsilon)

    # Apply steering limits
    delta = np.clip(delta, -delta_max, delta_max)

    # ---------------- Velocity Control ----------------

    # Reduce speed near final target
    dist_to_end = np.linalg.norm(np.array([x, y]) - path_pose[-1, 0:2], ord=2)
    ang_to_end = abs(np.rad2deg(pose[2] - path_pose[-1, 2]))

    if dist_to_end < 10.0 and ang_to_end < 60.0:
        vel_cmd = dist_to_end / 10.0 * vel_cmd
    elif dist_to_end < 1.0 and ang_to_end < 60.0:
        vel_cmd = 0.0

    # Reduce speed during sharp turns
    if abs(delta) >= 0.90 * delta_max:
        vel_cmd = min(vel_cmd, 0.04)
    elif abs(delta) >= 0.80 * delta_max:
        vel_cmd = min(vel_cmd, 0.06)
    elif abs(delta) >= 0.70 * delta_max:
        vel_cmd = min(vel_cmd, 0.08)
    elif abs(delta) >= 0.60 * delta_max:
        vel_cmd = min(vel_cmd, 0.10)
    elif abs(delta) >= 0.50 * delta_max:
        vel_cmd = min(vel_cmd, 0.15)

    return vel_cmd, delta


# =============================================================================
# Main Execution Configuration
# =============================================================================

# region: Experiment constants
simulationTime = 1200   # Total simulation duration (seconds)
frequency = 200         # Control loop frequency (Hz)
frameRate = 30          # Camera frame rate
CameraCounts = int(round(frequency / frameRate))
useCameras = False      # Enable/disable camera streams
# endregion 


# =============================================================================
# State Variables and Initialization
# =============================================================================

counter = 0
receiveCounter = 0
receivedData = np.zeros(10)

receiveGameCounter = 0
pose = np.zeros(3)

# Load initial spawn position and planned paths
initial_position = read_initial_positions(Path("spawn_locations.txt"))
pathposes_path = Path(r"tools\QCar2_PathPlanning\qcar2_pathposes.npy")
pathposes = load_plan_file(pathposes_path)


# =============================================================================
# Camera Initialization (Optional)
# =============================================================================
# Cameras simulate multiple viewpoints around the vehicle
if useCameras:
    camRight = Camera2D(cameraId="0@tcpip://localhost:18961", frameWidth=640, frameHeight=480, frameRate=frameRate)
    camBack  = Camera2D(cameraId="1@tcpip://localhost:18962", frameWidth=640, frameHeight=480, frameRate=frameRate)
    camLeft  = Camera2D(cameraId="3@tcpip://localhost:18964", frameWidth=640, frameHeight=480, frameRate=frameRate)
    camFront = Camera2D(cameraId="2@tcpip://localhost:18963", frameWidth=640, frameHeight=480, frameRate=frameRate)


# =============================================================================
# Communication Streams Setup
# =============================================================================

# Main data stream (control commands and telemetry)
dataStream = BasicStream(
    'tcpip://localhost:18375',
    agent='C',
    sendBufferSize=1460,
    receiveBuffer=np.zeros((1,10), dtype=np.float64),
    recvBufferSize=1460,
    nonBlocking=False
)

# Client stream for receiving vehicle pose
client_car = BasicStream(
    'tcpip://localhost:19000',
    agent='C',
    sendBufferSize=8,
    receiveBuffer=np.zeros((1,3), dtype=np.float64),
    recvBufferSize=24,
    nonBlocking=False
)

timeout = Timeout(seconds=0, nanoseconds=10)


# =============================================================================
# Control Loop Initialization
# =============================================================================

timer = QTimer(frequency, simulationTime)

# Flags and control variables
flag_send_intention = True
intention = 0
send_commands = np.array([0., 0.], dtype=np.float64)

# Keyboard input handling (runs in background thread)
user_mode = "task1"
mode_queue = queue.Queue()

input_thread = threading.Thread(
    target=keyboard_listener,
    args=(mode_queue,),
    daemon=True,
)
input_thread.start()


# =============================================================================
# Main Control Loop
# =============================================================================

try:
    while timer.check():

        current_time = timer.get_current_time()

        # ---------------- Mode Switching ----------------
        # Check if user changed driving mode
        while not mode_queue.empty():
            new_mode = mode_queue.get()

            if new_mode != user_mode:
                user_mode = new_mode
                flag_send_intention = True

                if user_mode == "task1":
                    print("Switched to task 1: Stop.")
                elif user_mode == "task2":
                    print("Switched to task 2: Go to Central Pickup Location and pick small packages.")

        # ---------------- Mode Behavior ----------------
        # Assign path and velocity based on selected mode
        if user_mode == "task1":
            intention = 0
            vel_cmd = 0.
            path_pose = pathposes[8, 8]
        elif user_mode == "task2":
            vel_cmd = 0.1
            intention = 1
            path_pose = pathposes[8, 24]

        # ---------------- Client Communication ----------------
        # Ensure connection and receive vehicle pose
        if not client_car.connected:
            client_car.checkConnection(timeout=timeout)

        if client_car.connected:
            if flag_send_intention:
                client_car.send(np.array(intention, dtype=np.float64))
                flag_send_intention = False

            recvFlag, _ = client_car.receive(iterations=2, timeout=timeout)

            if not recvFlag:
                receiveGameCounter += 1
                if receiveGameCounter > 1000:
                    print('QCar stopped receiving GPS data.')
            else:
                receiveGameCounter = 0
                pose = client_car.receiveBuffer[0]

        # ---------------- Control Computation ----------------
        # Compute velocity and steering using Stanley controller
        vel_cmd, steering_cmd = stanley_controller(pose, vel_cmd, path_pose)
        send_commands = np.array([vel_cmd, steering_cmd], dtype=np.float64)

        # ---------------- Server Communication ----------------
        if not dataStream.connected:
            dataStream.checkConnection(timeout=timeout)

        if dataStream.connected:
            recvFlag, _ = dataStream.receive(iterations=2, timeout=timeout)

            if not recvFlag:
                receiveCounter += 1
                if receiveCounter > 10:
                    print('Client stopped sending data over.')
            else:
                receiveCounter = 0
                receivedData = dataStream.receiveBuffer[0]
                # Telemetry structure:
                # [0] Motor Power Consumption
                # [1] Battery Level
                # [2] Car Speed (m/s)
                # [3-5] Gyroscope data
                # [6-8] Accelerometer data
                # [9] Connection status flag

            # ---------------- Camera Handling ----------------
            if useCameras and counter % CameraCounts == 0:
                frameLeft = camLeft.read()
                frameRight = camRight.read()
                frameBack = camBack.read()
                frameFront = camFront.read()

                if frameLeft or frameRight or frameBack or frameFront:
                    cv2.imshow("Left Car Image", camLeft.imageData)
                    cv2.imshow("Right Car Image", camRight.imageData)
                    cv2.imshow("Back Car Image", camBack.imageData)
                    cv2.imshow("Front Car Image", camFront.imageData)
                    cv2.waitKey(1)

            counter += 1

            # ---------------- Command Transmission ----------------
            # Send computed velocity and steering commands
            sentFlag = dataStream.send(send_commands)
            if sentFlag == -1:
                print('Server application not receiving.')
                break

        # Maintain loop timing
        timer.sleep()

except KeyboardInterrupt:
    print("\nExiting due to keyboard interrupt.")


# =============================================================================
# Cleanup
# =============================================================================

if useCameras:
    camLeft.terminate()
    camBack.terminate()
    camRight.terminate()
    camFront.terminate()

dataStream.terminate()
client_car.terminate()