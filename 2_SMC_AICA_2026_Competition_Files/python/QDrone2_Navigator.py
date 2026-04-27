# =============================================================================
# Example Drone Navigator File
# -----------------------------------------------------------------------------
# This file is intended as a reference example. Competitors or developers can:
#   - Use it as a baseline for their own drone navigation systems
#   - Modify and extend it for improved performance or new features
#   - Integrate it into larger autonomous flight pipelines
#
# This script demonstrates a baseline implementation of a drone navigation and
# command system using waypoint interpolation, communication streams, and
# optional onboard camera feeds.
#
# The implementation includes:
#   - Fixed hover mode for simple position hold
#   - Time-parameterized waypoint trajectory execution
#   - TCP/IP communication with the simulation/client
#   - Optional multi-camera streaming
#   - Keyboard-based mode switching
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
from pal.utilities.vision import Camera2D, Camera3D
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
        np.ndarray: Drone initial pose values [x, y, z, yaw].
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

    # Drone pose is stored in the second group of 4 values
    return np.array(values[4:8], dtype=np.float64)


def keyboard_listener(mode_queue: queue.Queue):
    """
    Runs in a separate thread to listen for user keyboard input.

    Allows runtime switching between predefined drone modes.
    Input values:
        task 1 -> Hover at spawn location 
        task 2 -> Fly to Central Pickup Location and pick a small package up
    """
    print("Keyboard control is active.")
    print("Initial mode is task1.")
    print("Type task1 to hover at the spawn location.")
    print("Type task2 to command QDrone2 go to Central Pickup location and pick up a small package.")
    

    while True:
        try:
            user_input = input().strip()
        except EOFError:
            break

        if user_input in ("task1", "task2"):
            mode_queue.put(user_input)
        else:
            print("Invalid input. Enter only 1 or 2.")


def load_plan_file(plan_path: Path):
    """
    Loads precomputed drone waypoint trajectory data from a NumPy archive.

    Returns:
        tuple:
            qdrone2_wp1 (ndarray): Waypoint matrix
            qdrone2_t1  (ndarray): Time vector corresponding to waypoints
    """
    plan_data = np.load(plan_path, allow_pickle=True)

    qdrone2_wp1 = np.asarray(plan_data["qdrone2_wp1"])
    qdrone2_t1 = np.asarray(plan_data["qdrone2_t1"]).flatten()

    return qdrone2_wp1, qdrone2_t1


def interpolate_waypoint(
    t_query: float, t_vec: np.ndarray, wp_mat: np.ndarray
) -> np.ndarray:
    """
    Linearly interpolates the commanded waypoint at a requested time.

    Args:
        t_query: Query time
        t_vec: Monotonic time vector
        wp_mat: Waypoint matrix aligned with t_vec

    Returns:
        np.ndarray: Interpolated waypoint command
    """
    if t_query <= t_vec[0]:
        return wp_mat[0].astype(np.float64)

    if t_query >= t_vec[-1]:
        return wp_mat[-1].astype(np.float64)

    idx_right = np.searchsorted(t_vec, t_query, side="right")
    idx_left = idx_right - 1

    t0 = t_vec[idx_left]
    t1 = t_vec[idx_right]

    wp0 = wp_mat[idx_left]
    wp1 = wp_mat[idx_right]

    alpha = (t_query - t0) / (t1 - t0)

    return ((1.0 - alpha) * wp0 + alpha * wp1).astype(np.float64)


# =============================================================================
# Main Execution Configuration
# =============================================================================

# region: Experiment constants
simulationTime = 10000   # Total simulation duration (seconds)
frequency = 200          # Control loop frequency (Hz)
frameRate = 30           # Camera frame rate
CameraCounts = int(round(frequency / frameRate))
useCameras = False       # Enable/disable camera streams
# endregion


# =============================================================================
# State Variables and Initialization
# =============================================================================

counter = 0
receiveCounter = 0
receivedData = np.zeros(16)

# Load initial spawn pose and planned trajectory
initial_position = read_initial_positions(Path("spawn_locations.txt"))
plan_path = Path(r"tools\QDrone2_PathPlanning\qdrone2_plans.npz")
qdrone2_wp1, qdrone2_t1 = load_plan_file(plan_path)


# =============================================================================
# Camera Initialization (Optional)
# =============================================================================
# Cameras simulate multiple onboard viewpoints around the drone
if useCameras:
    realsense = Camera3D(
        deviceId="0@tcpip://localhost:18986",
        mode='RGB&DEPTH',
        frameWidthRGB=640,
        frameHeightRGB=480,
        frameRateRGB=frameRate,
        frameWidthDepth=640,
        frameHeightDepth=480,
        frameRateDepth=frameRate,
        readMode=0
    )

    camRight = Camera2D(
        cameraId="0@tcpip://localhost:18982",
        frameWidth=640,
        frameHeight=480,
        frameRate=frameRate
    )

    camBack = Camera2D(
        cameraId="1@tcpip://localhost:18983",
        frameWidth=640,
        frameHeight=480,
        frameRate=frameRate
    )

    camLeft = Camera2D(
        cameraId="2@tcpip://localhost:18984",
        frameWidth=640,
        frameHeight=480,
        frameRate=frameRate
    )

    camDown = Camera2D(
        cameraId="3@tcpip://localhost:18985",
        frameWidth=640,
        frameHeight=480,
        frameRate=frameRate
    )


# =============================================================================
# Communication Streams Setup
# =============================================================================

# Main data stream used to send position commands and receive telemetry
dataStream = BasicStream(
    'tcpip://localhost:18373',
    agent='C',
    sendBufferSize=1460,
    receiveBuffer=np.zeros((1, 20), dtype=np.float64),
    recvBufferSize=1460,
    nonBlocking=False
)

# Client stream used to communicate intention/mode information
client_drone = BasicStream(
    'tcpip://localhost:19001',
    agent='C',
    sendBufferSize=8,
    receiveBuffer=np.zeros((1, 3), dtype=np.float64),
    recvBufferSize=24,
    nonBlocking=False
)

timeout = Timeout(seconds=0, nanoseconds=10)
prev_con = False
prev_game_con = False


# =============================================================================
# Control Loop Initialization
# =============================================================================

timer = QTimer(frequency, simulationTime)

# Mode and command variables
intention = 0
flag_send_intention = True
send_commands = initial_position + np.array([0.0, 0.0, 3.0, 0.0], dtype=np.float64)

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
        # Check if the user selected a new flight mode
        while not mode_queue.empty():
            new_mode = mode_queue.get()

            if new_mode != user_mode:
                user_mode = new_mode
                flag_send_intention = True

                if user_mode == "task1":
                    intention = 0
                    print("Switched to mode 1: Hover at spawn location.")
                elif user_mode == "task2":
                    intention = 1
                    trajectory_start_time = None
                    trajectory_finished = False
                    print("Switched to mode 2: Fly to Central Pickup Location.")

        # ---------------- Mode Behavior ----------------
        # Assign position command based on selected mode
        if user_mode == "task1":
            # Hold a fixed hover point 3 meters above the initial drone spawn
            send_commands = initial_position + np.array(
                [0.0, 0.0, 3.0, 0.0], dtype=np.float64
            )
            intention = 0

        elif user_mode == "task2":
            # Follow the planned time-parameterized waypoint trajectory
            intention = 1

            if trajectory_start_time is None:
                trajectory_start_time = current_time

            elapsed_time = current_time - trajectory_start_time

            if elapsed_time <= qdrone2_t1[-1]:
                send_commands = interpolate_waypoint(
                    elapsed_time, qdrone2_t1, qdrone2_wp1
                )
            else:
                # After the trajectory ends, keep holding the final waypoint
                send_commands = qdrone2_wp1[-1].astype(np.float64)
                if not trajectory_finished:
                    print("Trajectory completed. Holding final waypoint.")
                    trajectory_finished = True

        # ---------------- Client Communication ----------------
        # Ensure connection and send intention when needed
        if not client_drone.connected:
            client_drone.checkConnection(timeout=timeout)

        if client_drone.connected:
            if flag_send_intention:
                client_drone.send(np.array(intention, dtype=np.float64))
                flag_send_intention = False

        # ---------------- Server Communication ----------------
        # Ensure connection to the drone/server data stream
        if not dataStream.connected:
            dataStream.checkConnection(timeout=timeout)

        # Execute the main telemetry and command exchange
        if dataStream.connected:
            recvFlag, bytesReceived = dataStream.receive(iterations=2, timeout=timeout)

            if not recvFlag:
                receiveCounter += 1
                if receiveCounter > 1000:
                    print('Client stopped sending data over.')
            else:
                receiveCounter = 0
                receivedData = dataStream.receiveBuffer[0]
                # Telemetry structure:
                # [0]     Stream connection flag
                # [1:4]   IMU gyroscope data (rad/s)
                # [4:7]   IMU accelerometer data (m/s^2)
                # [7:10]  Estimated angular position (rad)
                # [10:13] Estimated angular rates (rad/s)
                # [13:16] Estimated angular acceleration
                # [16:20] Pose x, y, z, yaw (m, m, m, rad)
                #             
            # ---------------- Camera Handling ----------------
            if useCameras and counter % CameraCounts == 0:
                frameLeft = camLeft.read()
                frameRight = camRight.read()
                frameBack = camBack.read()
                frameDown = camDown.read()
                realsense.read_RGB()
                realsense.read_depth()

                if frameLeft or frameRight or frameBack or frameDown:
                    imageLeft = camLeft.imageData
                    imageRight = camRight.imageData
                    imageBack = camBack.imageData
                    imageDown = camDown.imageData
                    imageRGB = realsense.imageBufferRGB
                    imageDepth = realsense.imageBufferDepthPX
                    # NOTE:
                    # imageDepth contains values mapped approximately from
                    # 0-255 over a depth range of about 0-9.44 meters.

                    cv2.imshow("Left Drone Image", imageLeft)
                    cv2.imshow("Right Drone Image", imageRight)
                    cv2.imshow("Back Drone Image", imageBack)
                    cv2.imshow("Downwards Drone Image", imageDown)
                    cv2.imshow("Front RGB Drone Image", imageRGB)
                    cv2.imshow("Front Depth Drone Image", imageDepth)

                    cv2.waitKey(1)

            counter += 1

            # ---------------- Command Transmission ----------------
            # Send the current position/yaw command to the drone
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
    realsense.terminate()
    camLeft.terminate()
    camBack.terminate()
    camRight.terminate()
    camDown.terminate()

dataStream.terminate()