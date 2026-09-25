# =============================================================================
# AICA Challenge Core Simulation File
# -----------------------------------------------------------------------------
# !!! DO NOT MODIFY OR CHANGE THIS FILE !!!
#
# This file defines the core logic of the AICA Challenge environment.
# It is responsible for:
#   - Managing vehicle interactions (QCar2 and QDrone2)
#   - Handling pickup, drop, and transfer logic
#   - Computing scores and tracking completion status
#   - Maintaining synchronization with external navigators
#
# Competitors MUST NOT modify this file. Any modification may:
#   - Break synchronization with evaluation systems
#   - Lead to inconsistent scoring
#   - Result in disqualification
#
# Users should instead implement their strategies within navigator files.
# =============================================================================

import os
import numpy as np

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.qdrone2 import QLabsQDrone2
from qvl.basic_shape import QLabsBasicShape
from qvl.system import QLabsSystem

from pal.utilities.timing import QTimer
from pal.utilities.stream import BasicStream
try:
    from quanser.common import Timeout
except:
    from quanser.communications import Timeout

# Vehicle physical parameters
WHEELBASE = 2.7  # QCar2 wheelbase (meters)

# Network configuration for communication with navigators
HOST = "127.0.0.1"
PORT_CAR = 19000
PORT_DRONE = 19001

# Timing constraints (seconds)
DURATION_PICKUP   = 3.0
DURATION_DROP     = 3.0
DURATION_TRANSFER = 3.0

# Base score assigned per completed delivery
CONSTANT_SCORE = 1000


# -------------------------------------------------------------------------
# Location Definitions (World Coordinates)
# -------------------------------------------------------------------------

# Pickup location (shared depot)
LOC_PICK = np.array([-2.50305, 29.6703, 0.05])

# Standard small drop locations (car + drone)
LOC_SMALL_DROPOFF_LIST = np.array([
    [11.2739, -10.84655, 0.05],
    [22.5478, 29.6703, 0.05],
    [0.0, 44.9735, 0.05],
    [-19.84125, 29.6703, 0.05]
])


# Bonus (window-level) drop locations (drone only)
LOC_SMALL_DROPOFF_WBONUS_LIST = np.array([
    np.array([11.2739, -10.84655, 0.05]) + np.array([3.9, -7.2, 9.6]),
    np.array([22.5478, 29.6703, 0.05]) + np.array([3.5, -12.9, 9.6]),
    np.array([0.0, 44.9735, 0.05]) + np.array([1.3, 2.0, 4.8])
])

# Bonus rewards associated with special drop locations
BONUS_LIST = 100 * np.array([4, 3, 2])

# Large package drop location (car only)
LOC_LARGE_DROPOFF = np.array([-12.8205, -4.5991, 0.05])

TIMEOUT=Timeout(seconds=0, nanoseconds=10)

# -------------------------------------------------------------------------
# Timing Configuration
# -------------------------------------------------------------------------
simulationTime = 1200  # Total simulation duration (seconds)
frequency = 10         # Control loop frequency (Hz)
timer = QTimer(frequency, simulationTime)

def get_actor_pose(actor):
    _, loc, rot, _ = actor.get_world_transform()
    return np.array(loc, dtype=float), np.array(rot, dtype=float)

def spawn_box(shape, actor_number, location, scale=(0.5, 0.5, 0.5)):
    shape.spawn_id(
        actorNumber=actor_number,
        location=location,
        rotation=[0, 0, 0],
        scale=list(scale),
        configuration=shape.SHAPE_CUBE,
        waitForConfirmation=False
    )
    shape.set_material_properties(color=[120 / 255, 60 / 255, 30 / 255], waitForConfirmation=False)
    shape.set_enable_dynamics(False, waitForConfirmation=False)
    shape.set_enable_collisions(False, waitForConfirmation=False)

def destroy_box(shape, actor_number):
    shape.actorNumber = actor_number
    shape.destroy()

def update_box_pose(shape, actor_number, location, yaw, z_offset, scale=(0.5, 0.5, 0.5)):
    shape.actorNumber = actor_number
    shape.set_transform(
        location=location + np.array([0.0, 0.0, z_offset]),
        rotation=[0, 0, yaw],
        scale=list(scale),
        waitForConfirmation=False
    )

def spawn_completion_markers(shape, objective_index):
    # Normal marker
    spawn_box(
        shape,
        actor_number=200 + objective_index,
        location=LOC_SMALL_DROPOFF_LIST[objective_index] + np.array([0.0, 0.0, 0.25]),
        scale=[0.5, 0.5, 0.5]
    )

    # Bonus marker if this objective has one
    if objective_index < len(LOC_SMALL_DROPOFF_WBONUS_LIST):
        spawn_box(
            shape,
            actor_number=220 + objective_index,
            location=LOC_SMALL_DROPOFF_WBONUS_LIST[objective_index] + np.array([0.0, 0.0, 0.25]),
            scale=[0.5, 0.5, 0.5]
        )

def show_score_time(qlabs_sys, score, time):
    minutes = int(time) // 60
    seconds = int(time) % 60
    qlabs_sys.set_title_string(f'AICA Challenge 2026 - Time: {minutes:02d}:{seconds:02d} - Score: {int(score)}.')

def show_score_time_game_over(qlabs_sys, score, time):
    minutes = int(time) // 60
    seconds = int(time) % 60
    qlabs_sys.set_title_string(f'AICA Challenge 2026 - Time: {minutes:02d}:{seconds:02d} - Score: {int(score)}. Game over!')


def main():
    qlabs = QuanserInteractiveLabs()

    print("Connecting to QLabs...")
    if not qlabs.open("localhost"):
        print("Unable to connect to QLabs")
        return
    print("Connected")

    qlabs_basic_shape = QLabsBasicShape(qlabs)
    qlabs_sys = QLabsSystem(qlabs)

    hQCar = QLabsQCar2(qlabs, True)
    hQCar.actorNumber = 0

    hQDrone = QLabsQDrone2(qlabs, True)
    hQDrone.actorNumber = 1

    server_car = BasicStream('tcpip://localhost:19000', 
                       agent='S', sendBufferSize=24, 
                       receiveBuffer=np.zeros(1, dtype=np.float64),
                       recvBufferSize=8, nonBlocking=True)
    
    
    server_drone = BasicStream('tcpip://localhost:19001', 
                       agent='S', sendBufferSize=24, 
                       receiveBuffer=np.zeros(1, dtype=np.float64),
                       recvBufferSize=8, nonBlocking=True)

    print(f"Listening for QCar2 on {HOST}:{PORT_CAR}...")
    print(f"Listening for QDrone2 on {HOST}:{PORT_DRONE}...")
    

    time_car_pick_small = 0.0
    time_car_pick_large = 0.0
    time_car_drop = np.zeros(5)

    time_drone_pick_small = 0.0
    time_drone_drop = np.zeros(4)
    time_drone_drop_wbonus = np.zeros(3)

    time_transfer_drone_to_car = 0.0
    time_transfer_car_to_drone = 0.0

    completed_deliveries = np.zeros(5)

    no_small_box_car = 0
    no_large_box_car = 0
    no_small_box_drone = 0

    score = 0.0

    current_time = timer.get_current_time()

    intention_car = 0
    intention_drone = 0    

    try:
        while timer.check():
            prev_time = current_time
            current_time = timer.get_current_time()
            dt = current_time - prev_time
            

            if np.floor(prev_time) < np.floor(current_time):
                show_score_time(qlabs_sys, score, current_time)

            # Get intentions from server -------------

            loc_car_rear, rot_car = get_actor_pose(hQCar)
            loc_drone, rot_drone = get_actor_pose(hQDrone)

            # Rear axle to front axle for better automatic control
            loc_car = np.array([loc_car_rear[0] + WHEELBASE*np.cos(rot_car[2]), 
                                loc_car_rear[1] + WHEELBASE*np.sin(rot_car[2]),
                                loc_car_rear[2]])

            # Send car locations -------------------
            if not server_car.connected:
                server_car.checkConnection(timeout=TIMEOUT)

            if server_car.connected:
                recvFlag, bytesReceived = server_car.receive(iterations=1, timeout=TIMEOUT)
                if recvFlag:
                    data = server_car.receiveBuffer[0]
                    intention_car = int(data)
                server_car.send(np.array([loc_car[0], loc_car[1], rot_car[2]], dtype=np.float64))

            
            if not server_drone.connected:
                server_drone.checkConnection(timeout=TIMEOUT)

            if server_drone.connected:
                recvFlag, bytesReceived = server_drone.receive(iterations=1, timeout=TIMEOUT)
                if recvFlag:
                    data = server_drone.receiveBuffer[0]
                    intention_drone = int(data)

            # Update box poses on car
            if no_small_box_car > 0:
                update_box_pose(
                    qlabs_basic_shape,
                    actor_number=100,
                    location=loc_car_rear,
                    yaw=rot_car[2],
                    z_offset=2.0,
                    scale=[0.5, 0.5, 0.5]
                )

            if no_small_box_car > 1:
                update_box_pose(
                    qlabs_basic_shape,
                    actor_number=101,
                    location=loc_car_rear,
                    yaw=rot_car[2],
                    z_offset=2.52,
                    scale=[0.5, 0.5, 0.5]
                )

            if no_large_box_car == 1:
                update_box_pose(
                    qlabs_basic_shape,
                    actor_number=102,
                    location=loc_car_rear,
                    yaw=rot_car[2],
                    z_offset=2.0,
                    scale=[1.0, 1.0, 1.0]
                )

            # Update box pose on drone
            if no_small_box_drone == 1:
                update_box_pose(
                    qlabs_basic_shape,
                    actor_number=103,
                    location=loc_drone,
                    yaw=rot_drone[2],
                    z_offset=-0.3,
                    scale=[0.5, 0.5, 0.5]
                )

            # =========================================================
            # Car logic
            # =========================================================
            if intention_car == 3:  # Drop package
                time_car_pick_small = 0.0
                time_car_pick_large = 0.0

                if no_small_box_car > 0:
                    dropped_here = False
                    for i in range(4):
                        if (
                            np.linalg.norm(loc_car - LOC_SMALL_DROPOFF_LIST[i]) <= 2.0
                            and completed_deliveries[i] == 0
                        ):
                            time_car_drop[i] += dt
                            print(f"QCar2 Drop-off timer[{i}]: {time_car_drop[i]:.2f}")
                            dropped_here = True

                            if time_car_drop[i] >= DURATION_DROP:
                                completed_deliveries[i] = 1
                                score += CONSTANT_SCORE - current_time
                                show_score_time(qlabs_sys, score, current_time)

                                spawn_completion_markers(qlabs_basic_shape, i)

                                if no_small_box_car == 2:
                                    no_small_box_car = 1
                                    destroy_box(qlabs_basic_shape, actor_number=101)
                                elif no_small_box_car == 1:
                                    no_small_box_car = 0
                                    destroy_box(qlabs_basic_shape, actor_number=100)

                                time_car_drop[i] = 0.0
                            break
                        else:
                            time_car_drop[i] = 0.0

                    if not dropped_here:
                        time_car_drop[:4] = 0.0

                elif no_large_box_car == 1:
                    if (
                        np.linalg.norm(loc_car - LOC_LARGE_DROPOFF) <= 2.0
                        and completed_deliveries[4] == 0
                    ):
                        time_car_drop[4] += dt
                        print(f"QCar2 Drop-off timer[4]: {time_car_drop[4]:.2f}")

                        if time_car_drop[4] >= DURATION_DROP:
                            completed_deliveries[4] = 1
                            score += CONSTANT_SCORE - current_time
                            show_score_time(qlabs_sys, score, current_time)
                            no_large_box_car = 0
                            destroy_box(qlabs_basic_shape, actor_number=102)
                            time_car_drop[4] = 0.0

                            spawn_box(
                                qlabs_basic_shape,
                                actor_number=204,
                                location=LOC_LARGE_DROPOFF + np.array([0.0, 0.0, 0.5]),
                                scale=[1.0, 1.0, 1.0]
                            )
                    else:
                        time_car_drop[4] = 0.0
                else:
                    time_car_drop[:] = 0.0

            elif (
                intention_car == 1
                and np.linalg.norm(loc_car - LOC_PICK) <= 2.0
                and no_large_box_car == 0
                and no_small_box_car < 2
            ):
                time_car_pick_large = 0.0
                time_car_drop[:] = 0.0

                time_car_pick_small += dt
                print(
                    f"QCar2 pickup timer (small): {time_car_pick_small:.2f}, "
                    f"dist: {np.linalg.norm(loc_car - LOC_PICK):.2f}"
                )

                if time_car_pick_small >= DURATION_PICKUP:
                    time_car_pick_small = 0.0

                    if no_small_box_car == 0:
                        no_small_box_car = 1
                        spawn_box(
                            qlabs_basic_shape,
                            actor_number=100,
                            location=loc_car + np.array([0.0, 0.0, 2.0]),
                            scale=[0.5, 0.5, 0.5]
                        )
                    elif no_small_box_car == 1:
                        no_small_box_car = 2
                        spawn_box(
                            qlabs_basic_shape,
                            actor_number=101,
                            location=loc_car + np.array([0.0, 0.0, 2.52]),
                            scale=[0.5, 0.5, 0.5]
                        )

            elif (
                intention_car == 2
                and np.linalg.norm(loc_car - LOC_PICK) <= 2.0
                and no_large_box_car == 0
                and no_small_box_car == 0
            ):
                time_car_pick_small = 0.0
                time_car_drop[:] = 0.0

                time_car_pick_large += dt
                print(f"QCar2 pickup timer (large): {time_car_pick_large:.2f}")

                if time_car_pick_large >= DURATION_PICKUP:
                    time_car_pick_large = 0.0
                    no_large_box_car = 1
                    spawn_box(
                        qlabs_basic_shape,
                        actor_number=102,
                        location=loc_car + np.array([0.0, 0.0, 2.5]),
                        scale=[1.0, 1.0, 1.0]
                    )
            else:
                time_car_pick_small = 0.0
                time_car_pick_large = 0.0
                if intention_car != 3:
                    time_car_drop[:] = 0.0

            # =========================================================
            # Drone logic
            # =========================================================
            if intention_drone == 1:  # Drone pickup
                time_drone_drop[:] = 0.0
                time_drone_drop_wbonus[:] = 0.0

                dx = loc_drone[0] - LOC_PICK[0]
                dy = loc_drone[1] - LOC_PICK[1]
                dz = loc_drone[2] - LOC_PICK[2]

                if (
                    np.hypot(dx, dy) <= 2.0
                    and 0.0 <= dz <= 4.0
                    and no_small_box_drone == 0
                ):
                    time_drone_pick_small += dt
                    print(f"QDrone2 pickup timer: {time_drone_pick_small:.2f}")

                    if time_drone_pick_small >= DURATION_PICKUP:
                        time_drone_pick_small = 0.0
                        no_small_box_drone = 1
                        spawn_box(
                            qlabs_basic_shape,
                            actor_number=103,
                            location=loc_drone + np.array([0.0, 0.0, -0.3]),
                            scale=[0.5, 0.5, 0.5]
                        )
                else:
                    time_drone_pick_small = 0.0
            elif intention_drone == 2:  # Drone drop
                time_drone_pick_small = 0.0

                if no_small_box_drone == 1:
                    bonus_match = False

                    # Bonus/window drops
                    for i in range(len(LOC_SMALL_DROPOFF_WBONUS_LIST)):
                        dx = loc_drone[0] - LOC_SMALL_DROPOFF_WBONUS_LIST[i][0]
                        dy = loc_drone[1] - LOC_SMALL_DROPOFF_WBONUS_LIST[i][1]
                        dz = loc_drone[2] - LOC_SMALL_DROPOFF_WBONUS_LIST[i][2]

                        if (
                            np.hypot(dx, dy) <= 2.0
                            and 0.0 <= dz <= 4.0
                            and completed_deliveries[i] == 0
                        ):
                            time_drone_drop_wbonus[i] += dt
                            print(f"QDrone2 window delivery drop-off timer[{i}]: {time_drone_drop_wbonus[i]:.2f}")
                            bonus_match = True

                            if time_drone_drop_wbonus[i] >= DURATION_DROP:
                                completed_deliveries[i] = 1
                                score += CONSTANT_SCORE - current_time + BONUS_LIST[i]
                                show_score_time(qlabs_sys, score, current_time)
                                no_small_box_drone = 0
                                destroy_box(qlabs_basic_shape, actor_number=103)
                                time_drone_drop_wbonus[i] = 0.0

                                spawn_completion_markers(qlabs_basic_shape, i)
                            break
                        else:
                            time_drone_drop_wbonus[i] = 0.0

                    if not bonus_match:
                        time_drone_drop_wbonus[:] = 0.0

                        # Standard small drop locations
                        normal_match = False
                        for i in range(4):
                            dx = loc_drone[0] - LOC_SMALL_DROPOFF_LIST[i][0]
                            dy = loc_drone[1] - LOC_SMALL_DROPOFF_LIST[i][1]
                            dz = loc_drone[2] - LOC_SMALL_DROPOFF_LIST[i][2]

                            if (
                                np.hypot(dx, dy) < 0.5
                                and 0.0 <= dz <= 4.0
                                and completed_deliveries[i] == 0
                            ):
                                time_drone_drop[i] += dt
                                print(f"QDrone2 drop-off timer[{i}]: {time_drone_drop[i]:.2f}")
                                normal_match = True

                                if time_drone_drop[i] >= DURATION_DROP:
                                    completed_deliveries[i] = 1
                                    score += CONSTANT_SCORE - current_time
                                    show_score_time(qlabs_sys, score, current_time)
                                    no_small_box_drone = 0
                                    destroy_box(qlabs_basic_shape, actor_number=103)
                                    time_drone_drop[i] = 0.0

                                    spawn_completion_markers(qlabs_basic_shape, i)
                                break
                            else:
                                time_drone_drop[i] = 0.0

                        if not normal_match:
                            time_drone_drop[:] = 0.0
                    else:
                        time_drone_drop[:] = 0.0
                else:
                    time_drone_drop[:] = 0.0
                    time_drone_drop_wbonus[:] = 0.0

            else:
                time_drone_pick_small = 0.0
                if intention_drone != 2:
                    time_drone_drop[:] = 0.0
                    time_drone_drop_wbonus[:] = 0.0

            # =========================================================
            # Transfer Logic
            # =========================================================
            # Handles package transfer between:
            #   - Car -> Drone
            #   - Drone -> Car
            if intention_drone == 3 and intention_car == 5:  # Car -> Drone
                dx = loc_drone[0] - loc_car[0]
                dy = loc_drone[1] - loc_car[1]
                dz = loc_drone[2] - loc_car[2]

                if (
                    np.hypot(dx, dy) < 1.2
                    and 0.0 <= dz <= 4.0
                    and no_small_box_drone == 0
                    and no_small_box_car > 0
                ):
                    time_transfer_car_to_drone += dt
                    print(f"QCar2-to-QDrone2 package transfer timer: {time_transfer_car_to_drone:.2f}")

                    if time_transfer_car_to_drone >= DURATION_TRANSFER:
                        time_transfer_car_to_drone = 0.0

                        if no_small_box_car == 2:
                            no_small_box_car = 1
                            destroy_box(qlabs_basic_shape, actor_number=101)
                        elif no_small_box_car == 1:
                            no_small_box_car = 0
                            destroy_box(qlabs_basic_shape, actor_number=100)

                        no_small_box_drone = 1
                        spawn_box(
                            qlabs_basic_shape,
                            actor_number=103,
                            location=loc_drone + np.array([0.0, 0.0, -0.3]),
                            scale=[0.5, 0.5, 0.5]
                        )
                else:
                    time_transfer_car_to_drone = 0.0
            else:
                time_transfer_car_to_drone = 0.0

            if intention_drone == 4 and intention_car == 4:  # Drone -> Car
                dx = loc_drone[0] - loc_car[0]
                dy = loc_drone[1] - loc_car[1]
                dz = loc_drone[2] - loc_car[2]

                if (
                    np.hypot(dx, dy) < 1.2
                    and 0.0 <= dz <= 4.0
                    and no_small_box_drone == 1
                    and no_large_box_car == 0
                    and no_small_box_car < 2
                ):
                    time_transfer_drone_to_car += dt
                    print(f"QDrone2-to-QCar2 package transfer timer: {time_transfer_drone_to_car:.2f}")

                    if time_transfer_drone_to_car >= DURATION_TRANSFER:
                        time_transfer_drone_to_car = 0.0

                        no_small_box_drone = 0
                        destroy_box(qlabs_basic_shape, actor_number=103)

                        if no_small_box_car == 0:
                            no_small_box_car = 1
                            spawn_box(
                                qlabs_basic_shape,
                                actor_number=100,
                                location=loc_car + np.array([0.0, 0.0, 2.0]),
                                scale=[0.5, 0.5, 0.5]
                            )
                        elif no_small_box_car == 1:
                            no_small_box_car = 2
                            spawn_box(
                                qlabs_basic_shape,
                                actor_number=101,
                                location=loc_car + np.array([0.0, 0.0, 2.5]),
                                scale=[0.5, 0.5, 0.5]
                            )
                else:
                    time_transfer_drone_to_car = 0.0
            else:
                time_transfer_drone_to_car = 0.0

                    # =========================================================
        # Completion Check (Game Termination Condition)
        # =========================================================
        # Check if all deliveries are completed
        if np.sum(completed_deliveries) == 5:
            qlabs_sys.set_title_string(
                f'AICA Challenge 2026. Score: {int(score)}. All deliveries are completed!'
            )
            print("All deliveries are completed!")

        timer.sleep()

    finally:
        server_car.terminate()
        server_drone.terminate()
        qlabs.close()


if __name__ == "__main__":
    os.system("cls")
    main()