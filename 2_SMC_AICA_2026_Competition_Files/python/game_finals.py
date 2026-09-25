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
from collections import deque

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

ROAD_Z_WINDOW   = 200    # samples (10Hz -> 20s) for the road-level median
SPEED_SMOOTH    = 3      # samples averaged before differentiating speed
JOLT_TICKS      = 2      # consecutive ticks above threshold to count
HIT_CLEAR_TIME  = 0.60   # s back on the road before a new hit can register


HIT_CLEAR_DIST  = 3.50   # m travelled clear before a new hit can register


curb_REPEAT_TIME = 1.0   # s of continuous contact per additional hit

PENALTY_PER_HIT = 30.0    # score deducted per hit, applied ONCE AT THE END

# Radius around each pickup and drop-off point used by the legacy

STOP_ZONE_R     = 4.0    # m around a pickup/drop


GRID_FILE       = "drivable_grid.npz"
CAR_HALF_TRACK  = 0.80   # m from centreline to each wheel
curb_MIN_WHEELS = 1      # wheels off the road that constitutes a hit

curb_MIN_DEPTH  = 0.10   # m past the road edge before it counts
curb_MIN_TICKS  = 1      # consecutive qualifying ticks before it counts


curb_TRACE      = True


SCORE_TIME_CAP = 300.0


def delivery_score(current_time: float, bonus: float = 0.0) -> float:
    """Score contribution for one completed delivery. Zero once the run
    passes SCORE_TIME_CAP -- anything delivered after that point simply
    doesn't move the score."""
    if current_time > SCORE_TIME_CAP:
        return 0.0
    return CONSTANT_SCORE - current_time + bonus

URGENT_WINDOW_TIME_LIMIT = 120.0  # 2 minutes
URGENT_WINDOW_BONUS = 200.0
DEFAULT_MARKER_COLOR = [120 / 255, 60 / 255, 30 / 255]


# -------------------------------------------------------------------------
# Location Definitions (World Coordinates)
# -------------------------------------------------------------------------

# Pickup location (shared depot)
LOC_PICK = np.array([-2.50305, 29.6703, 0.05])


BUILDINGS = [
    {   # Building 1 -- the 3-window building
        "name": "Building 1", "slots": 3,
        "ground": np.array([11.2739, -10.84655, 0.05]),          # Common 1
        "windows": [
            {"label": 1, "loc": np.array([15.1739, -18.04655, 9.65]), "bonus": 400},
            {"label": 2, "loc": np.array([-3.492, -18.441, 9.663]),   "bonus": 400},
            {"label": 3, "loc": np.array([3.475, -21.886, 11.291]),   "bonus": 400},
        ],
    },
    {   # Building 2 -- one common, one window
        "name": "Building 2", "slots": 1,
        "ground": np.array([22.435, 1.392, 0.005]),              # Common 2
        "windows": [
            {"label": 4, "loc": np.array([27.787, 0.358, 4.291]), "bonus": 200},
        ],
    },
    {   # Building 3 -- one common, two windows
        "name": "Building 3", "slots": 2,
        "ground": np.array([22.5478, 29.6703, 0.05]),            # Common 3
        "windows": [
            {"label": 5, "loc": np.array([25.401, 17.005, 9.15]),  "bonus": 300},
            {"label": 6, "loc": np.array([25.32, 35.156, 13.321]), "bonus": 400},
        ],
    },
    {   # Building 4 -- one common, two windows
        "name": "Building 4", "slots": 2,
        "ground": np.array([0.0, 44.9735, 0.05]),                # Common 4
        "windows": [
            {"label": 7, "loc": np.array([1.3, 47.5985, 4.807]), "bonus": 100},
            {"label": 8, "loc": np.array([-10.8, 47.34, 4.357]), "bonus": 100},
        ],
    },
    {   # Building 5 -- ground only, no window
        "name": "Building 5", "slots": 1,
        "ground": np.array([-19.84125, 29.6703, 0.05]),          # Common 5
        "windows": [],
    },
    {   # Building 6 -- window only (drone)
        "name": "Building 6", "slots": 1,
        "ground": None,
        "windows": [
            {"label": 9, "loc": np.array([10.804, 27.047, 4.265]), "bonus": 200},
        ],
    },
    {   # Building 7 -- window only (drone), URGENT
        "name": "Building 7", "slots": 1,
        "ground": None,
        "windows": [
            {"label": 10, "loc": np.array([-23.873, 6.209, 9.15]), "bonus": 300},
        ],
    },
]

# Flat window table -- index here is what window_used / window_spawned use.
WINDOWS = []
for _b, _spec in enumerate(BUILDINGS):
    for _w in _spec["windows"]:
        WINDOWS.append({"building": _b, **_w})

# Window display labels that pay the extra early-bird bonus.
URGENT_WINDOW_LABELS = {10}

TOTAL_BUILDING_SLOTS = sum(b["slots"] for b in BUILDINGS)

LOC_LARGE_DROPOFF_LIST = np.array([
    [-12.8205, -4.5991, 0.05],    # 0 - Large 1
    [8.975, 37.099, 0.005],       # 1 - Large 2
    [8.367, 10.853, 0.005],       # 2 - Large 3
])

# Everywhere the car deliberately comes to a stop (see STOP_ZONE_R).
STOP_ZONES = np.array(
    [LOC_PICK[:2]]
    + [b["ground"][:2] for b in BUILDINGS if b["ground"] is not None]
    + [L[:2] for L in LOC_LARGE_DROPOFF_LIST]
)


def near_stop_zone(x, y):
    # True where the car is expected to brake hard on purpose.
    return bool(np.any(np.hypot(STOP_ZONES[:, 0] - x, STOP_ZONES[:, 1] - y)
                       <= STOP_ZONE_R))


TIMEOUT=Timeout(seconds=0, nanoseconds=100000)

# -------------------------------------------------------------------------
# Timing Configuration
# -------------------------------------------------------------------------
simulationTime = 1200  # Total simulation duration (seconds)
frequency = 10         # Control loop frequency (Hz)
timer = QTimer(frequency, simulationTime)

def get_actor_pose(actor): 
    _, loc, rot, _ = actor.get_world_transform()
    return np.array(loc, dtype=float), np.array(rot, dtype=float)

def spawn_box(shape, actor_number, location, scale=(0.5, 0.5, 0.5), color=None):
    shape.spawn_id(
        actorNumber=actor_number,
        location=location,
        rotation=[0, 0, 0],
        scale=list(scale),
        configuration=shape.SHAPE_CUBE,
        waitForConfirmation=False
    )
    shape.set_material_properties(color=list(color) if color is not None else DEFAULT_MARKER_COLOR, waitForConfirmation=False)
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

def reveal_building(shape, b, building_filled, window_used,
                    ground_spawned, window_spawned, current_time):
    spec = BUILDINGS[b]
    full = building_filled[b] >= spec["slots"]

    def put_ground():
        if spec["ground"] is None or ground_spawned[b] == 1:
            return
        ground_spawned[b] = 1
        spawn_box(shape, actor_number=200 + b,
                  location=spec["ground"] + np.array([0.0, 0.0, 0.25]),
                  scale=[0.5, 0.5, 0.5])

    def put_window(wi):
        if window_spawned[wi] == 1:
            return
        window_spawned[wi] = 1
        spawn_box(shape, actor_number=230 + wi,
                  location=WINDOWS[wi]["loc"] + np.array([0.0, 0.0, 0.25]),
                  scale=[0.5, 0.5, 0.5])

    if full:
        put_ground()
        for wi, w in enumerate(WINDOWS):
            if w["building"] == b:
                put_window(wi)
    elif current_time > SCORE_TIME_CAP:
        if building_filled[b] > 0:
            put_ground()
        for wi, w in enumerate(WINDOWS):
            if w["building"] == b and window_used[wi] == 1:
                put_window(wi)


def spawn_window_pad(shape, wi, window_spawned):

    if window_spawned[wi] == 1:
        return
    window_spawned[wi] = 1
    spawn_box(shape, actor_number=230 + wi,
              location=WINDOWS[wi]["loc"] + np.array([0.0, 0.0, 0.25]),
              scale=[0.5, 0.5, 0.5])


class DrivableGrid:


    def __init__(self):
        self.ok = False
        self.mask = None
        self.depth = None
        self.x0 = self.y0 = 0.0
        self.cell = 1.0

    def load(self, path):
        try:
            data = np.load(path)
            self.mask = data["mask"]
            self.depth = data["depth"] if "depth" in data else None
            self.x0 = float(data["x0"])
            self.y0 = float(data["y0"])
            self.cell = float(data["cell"])
        except Exception as exc:
            print(f"[curb] no drivable grid ({exc}).")
            print(f"[curb] run build_drivable_grid.py first -- "
                  f"curb detection is OFF for this run.")
            return False
        self.ok = True
        h, w = self.mask.shape
        cov = float(data["lane_coverage"]) if "lane_coverage" in data else -1.0
        #print(f"[curb] drivable grid loaded: {w} x {h} cells at "
              #f"{self.cell:.2f} m"
             #         + (f", lane coverage {cov:.1f}%" if cov >= 0 else ""))
        return True

    def is_drivable(self, x, y):
        """True where the point is on road. Off the grid entirely = off road."""
        i = np.round((np.asarray(x) - self.x0) / self.cell).astype(int)
        j = np.round((np.asarray(y) - self.y0) / self.cell).astype(int)
        h, w = self.mask.shape
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        out = np.zeros(np.shape(i), dtype=bool)
        out[inside] = self.mask[j[inside], i[inside]]
        return out

    def wheels_off(self, x_rear, y_rear, yaw):
        """Number of wheels off the road surface, 0 to 4."""
        return self.wheels_state(x_rear, y_rear, yaw)[0]

    def wheels_state(self, x_rear, y_rear, yaw):
  
        if not self.ok:
            return 0, 0.0
        c, s = np.cos(yaw), np.sin(yaw)
        # along-axis offsets from the rear axle, and across-axis offsets
        along = np.array([0.0, 0.0, WHEELBASE, WHEELBASE])
        across = np.array([-CAR_HALF_TRACK, CAR_HALF_TRACK,
                           -CAR_HALF_TRACK, CAR_HALF_TRACK])
        xs = x_rear + along * c - across * s
        ys = y_rear + along * s + across * c

        off = int(np.count_nonzero(~self.is_drivable(xs, ys)))
        if off == 0 or self.depth is None:
            # Grid files without a depth field fall back to counting only;
            # any crossing is then treated as qualifying.
            return off, (999.0 if off and self.depth is None else 0.0)

        i = np.round((xs - self.x0) / self.cell).astype(int)
        j = np.round((ys - self.y0) / self.cell).astype(int)
        h, w = self.mask.shape
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        d = np.full(4, 999.0)               # off the grid entirely = way out
        d[inside] = self.depth[j[inside], i[inside]]
        return off, float(d.max())


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

    # curb detection. Generated offline by build_drivable_grid.py; if the
    # file is absent the run proceeds without the curb penalty.
    grid = DrivableGrid()
    grid.load(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           GRID_FILE))
    

    time_car_pick_small = 0.0
    time_car_pick_large = 0.0
    time_car_drop_large = np.zeros(len(LOC_LARGE_DROPOFF_LIST))
    time_drone_pick_small = 0.0

    time_car_ground = np.zeros(len(BUILDINGS))
    time_drone_ground = np.zeros(len(BUILDINGS))
    time_window_drop = np.zeros(len(WINDOWS))

    time_transfer_drone_to_car = 0.0
    time_transfer_car_to_drone = 0.0

    building_filled = np.zeros(len(BUILDINGS), dtype=int)
    window_used = np.zeros(len(WINDOWS), dtype=int)
    completed_large = np.zeros(len(LOC_LARGE_DROPOFF_LIST))

    # Which pads already have their completion box down.
    ground_spawned = np.zeros(len(BUILDINGS), dtype=int)
    window_spawned = np.zeros(len(WINDOWS), dtype=int)

    TOTAL_DELIVERIES = TOTAL_BUILDING_SLOTS + len(LOC_LARGE_DROPOFF_LIST)

    no_small_box_car = 0
    no_large_box_car = 0
    no_small_box_drone = 0

    score = 0.0


    curb_hits = 0
    curb_log = []           # (t, x, y, detail) per hit, reported at the end
    end_reason = "time limit"
    end_time = 0.0
    road_z_hist = deque(maxlen=ROAD_Z_WINDOW)   # recent car z, for the median
    road_z_ref = 0.0        # median car z == road level
    prev_xy = None          # last tick's position, for speed
    speed_hist = deque(maxlen=SPEED_SMOOTH)   # smoothed before differentiating
    prev_speed = 0.0
    prev_yaw = None
    _jolt_ticks = 0         # consecutive ticks with a jolt, vs JOLT_TICKS
    max_wheels_off = 0      # worst excursion seen, for the summary
    max_depth = 0.0         # furthest any wheel got past the curb line
    _off_ticks = 0          # consecutive ticks deep enough to count
    near_misses = 0         # excursions too brief to qualify
    shallow_ticks = 0       # ticks off the road but not deep enough
    curb_trace = []         # (t, x, y, yaw, wheels_off, depth) per tick
    max_rise = 0.0          # worst of each signal, for tuning
    max_decel = 0.0
    max_yawrate = 0.0
    worst_at = (0.0, 0.0, 0.0)
    _quiet_since = None     # when the vehicle last returned to the road
    _hit_armed = True       # cleared after a hit until the vehicle returns
    _last_hit_at = None     # last recorded hit, for curb_REPEAT_TIME
    _clear_dist = 0.0       # distance travelled since returning to the road
    _clear_prev = (0.0, 0.0)  # previous position while clear
    # Initialised here because they are referenced before assignment on the
    # first iteration.
    speed = decel = yaw_rate = rise = 0.0
    in_stop_zone = False

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

            # ---- curb / impact check --------------------------------
            car_z = float(loc_car_rear[2])
            road_z_hist.append(car_z)
            road_z_ref = float(np.median(road_z_hist))
            rise = car_z - road_z_ref

            speed = 0.0
            decel = 0.0
            yaw_rate = 0.0
            if prev_xy is not None and dt > 1e-6:
                raw_speed = float(np.hypot(loc_car_rear[0] - prev_xy[0],
                                           loc_car_rear[1] - prev_xy[1])) / dt
                speed_hist.append(raw_speed)
                speed = float(np.mean(speed_hist))   # smooth, then differentiate
                decel = max(0.0, (prev_speed - speed) / dt)
                dyaw = (float(rot_car[2]) - prev_yaw + np.pi) % (2 * np.pi) - np.pi
                yaw_rate = abs(dyaw) / dt
            prev_xy = (loc_car_rear[0], loc_car_rear[1])
            prev_speed = speed
            prev_yaw = float(rot_car[2])

            if rise > max_rise:
                max_rise = rise
                worst_at = (current_time, loc_car_rear[0], loc_car_rear[1])
            max_decel = max(max_decel, decel)
            max_yawrate = max(max_yawrate, yaw_rate)

            # curb test: place the four wheels according to the vehicle
            # pose and look each one up in the drivable grid.
            off, depth = grid.wheels_state(loc_car_rear[0], loc_car_rear[1],
                                           rot_car[2])
            max_wheels_off = max(max_wheels_off, off)
            if depth < 900.0:
                max_depth = max(max_depth, depth)

            if curb_TRACE:
                curb_trace.append((current_time, float(loc_car_rear[0]),
                                   float(loc_car_rear[1]), float(rot_car[2]),
                                   off, min(depth, 999.0)))

            # A tick qualifies only if enough wheels are off the road AND
            # the furthest is past curb_MIN_DEPTH.
            if off >= curb_MIN_WHEELS and depth >= curb_MIN_DEPTH:
                _off_ticks += 1
            else:

                if 0 < _off_ticks < curb_MIN_TICKS:
                    near_misses += 1
                if off > 0 and depth < curb_MIN_DEPTH:
                    shallow_ticks += 1
                _off_ticks = 0

            struck = _off_ticks >= curb_MIN_TICKS
            why_grid = (f"{off} wheel{'s' if off != 1 else ''} off road, "
                        f"{depth:.2f} m over" if struck else "")

            if struck:
                _quiet_since = None
                # Record on first contact of an excursion, and again for
                # each further curb_REPEAT_TIME spent off the road.
                take_hit = _hit_armed or (
                    _last_hit_at is not None
                    and current_time - _last_hit_at >= curb_REPEAT_TIME)
                if take_hit:
                    curb_hits += 1
                    _hit_armed = False
                    _last_hit_at = current_time
                    curb_log.append((current_time, float(loc_car_rear[0]),
                                     float(loc_car_rear[1]), why_grid))
                    print(f"[curb] hit #{curb_hits} at t={current_time:.1f}s -- "
                          f"{why_grid} at "
                          f"({loc_car_rear[0]:.2f}, {loc_car_rear[1]:.2f})")
            else:

                if _quiet_since is None:
                    _quiet_since = current_time
                    _clear_dist = 0.0
                else:
                    _clear_dist += float(np.hypot(
                        loc_car_rear[0] - _clear_prev[0],
                        loc_car_rear[1] - _clear_prev[1]))
                    if (current_time - _quiet_since >= HIT_CLEAR_TIME
                            and _clear_dist >= HIT_CLEAR_DIST):
                        _hit_armed = True
                _clear_prev = (float(loc_car_rear[0]), float(loc_car_rear[1]))

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
                    for bi, spec in enumerate(BUILDINGS):
                        if spec["ground"] is None:
                            continue
                        if (
                            np.linalg.norm(loc_car - spec["ground"]) <= 2.0
                            and building_filled[bi] < spec["slots"]
                        ):
                            time_car_ground[bi] += dt
                            print(f"QCar2 ground drop-off timer [{spec['name']}]: "
                                  f"{time_car_ground[bi]:.2f}")
                            dropped_here = True

                            if time_car_ground[bi] >= DURATION_DROP:
                                building_filled[bi] += 1
                                score += delivery_score(current_time)
                                show_score_time(qlabs_sys, score, current_time)
                                print(f"{spec['name']}: ground delivery "
                                      f"{building_filled[bi]}/{spec['slots']}")

                                reveal_building(
                                    qlabs_basic_shape, bi, building_filled, window_used,
                                    ground_spawned, window_spawned, current_time
                                )

                                if no_small_box_car == 2:
                                    no_small_box_car = 1
                                    destroy_box(qlabs_basic_shape, actor_number=101)
                                elif no_small_box_car == 1:
                                    no_small_box_car = 0
                                    destroy_box(qlabs_basic_shape, actor_number=100)

                                time_car_ground[bi] = 0.0
                            break
                        else:
                            time_car_ground[bi] = 0.0

                    if not dropped_here:
                        time_car_ground[:] = 0.0

                elif no_large_box_car == 1:
                    dropped_here = False
                    for j in range(len(LOC_LARGE_DROPOFF_LIST)):
                        if (
                            np.linalg.norm(loc_car - LOC_LARGE_DROPOFF_LIST[j]) <= 2.0
                            and completed_large[j] == 0
                        ):
                            time_car_drop_large[j] += dt
                            print(f"QCar2 Large Drop-off timer[{j}]: {time_car_drop_large[j]:.2f}")
                            dropped_here = True

                            if time_car_drop_large[j] >= DURATION_DROP:
                                completed_large[j] = 1
                                score += delivery_score(current_time)
                                show_score_time(qlabs_sys, score, current_time)
                                no_large_box_car = 0
                                destroy_box(qlabs_basic_shape, actor_number=102)
                                time_car_drop_large[j] = 0.0

                                spawn_box(
                                    qlabs_basic_shape,
                                    actor_number=210 + j,
                                    location=LOC_LARGE_DROPOFF_LIST[j] + np.array([0.0, 0.0, 0.5]),
                                    scale=[1.0, 1.0, 1.0]
                                )
                            break
                        else:
                            time_car_drop_large[j] = 0.0

                    if not dropped_here:
                        time_car_drop_large[:] = 0.0
                else:
                    time_car_ground[:] = 0.0
                    time_car_drop_large[:] = 0.0

            elif (
                intention_car == 1
                and np.linalg.norm(loc_car - LOC_PICK) <= 2.0
                and no_large_box_car == 0
                and no_small_box_car < 2
            ):
                time_car_pick_large = 0.0
                time_car_ground[:] = 0.0; time_car_drop_large[:] = 0.0

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
                time_car_ground[:] = 0.0; time_car_drop_large[:] = 0.0

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
                    time_car_ground[:] = 0.0; time_car_drop_large[:] = 0.0

            # =========================================================
            # Drone logic
            # =========================================================
            if intention_drone == 1:  # Drone pickup
                time_window_drop[:] = 0.0
                time_drone_ground[:] = 0.0

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
                    matched = False

                    # Window pads first -- one box each, pays that
                    # window's bonus on top of the base score. Only
                    # counts while the building still has a free slot.
                    for wi, w in enumerate(WINDOWS):
                        bi = w["building"]
                        spec = BUILDINGS[bi]
                        dx = loc_drone[0] - w["loc"][0]
                        dy = loc_drone[1] - w["loc"][1]
                        dz = loc_drone[2] - w["loc"][2]

                        if (
                            np.hypot(dx, dy) <= 2.0
                            and 0.0 <= dz <= 4.0
                            and window_used[wi] == 0
                            and building_filled[bi] < spec["slots"]
                        ):
                            time_window_drop[wi] += dt
                            print(f"QDrone2 window drop-off timer [Window {w['label']}]: "
                                  f"{time_window_drop[wi]:.2f}")
                            matched = True

                            if time_window_drop[wi] >= DURATION_DROP:
                                window_used[wi] = 1
                                building_filled[bi] += 1

                                bonus = w["bonus"]
                                if w["label"] in URGENT_WINDOW_LABELS:
                                    if current_time <= URGENT_WINDOW_TIME_LIMIT:
                                        bonus += URGENT_WINDOW_BONUS
                                        print(f"[URGENT] Window {w['label']} delivered at "
                                              f"t={current_time:.1f}s -- within "
                                              f"{URGENT_WINDOW_TIME_LIMIT:.0f}s limit, "
                                              f"+{URGENT_WINDOW_BONUS:.0f} bonus "
                                              f"(score with bonus {bonus:.0f}).")
                                    else:
                                        print(f"[URGENT] Window {w['label']} delivered at "
                                              f"t={current_time:.1f}s -- missed the "
                                              f"{URGENT_WINDOW_TIME_LIMIT:.0f}s window, "
                                              f"no urgent delivery bonus, score {bonus:.0f}.")

                                score += delivery_score(current_time, bonus)
                                show_score_time(qlabs_sys, score, current_time)
                                print(f"{spec['name']}: window delivery "
                                      f"{building_filled[bi]}/{spec['slots']}")
                                no_small_box_drone = 0
                                destroy_box(qlabs_basic_shape, actor_number=103)
                                time_window_drop[wi] = 0.0

                                # A window's own pad shows immediately.
                                spawn_window_pad(qlabs_basic_shape, wi, window_spawned)
                                reveal_building(
                                    qlabs_basic_shape, bi, building_filled, window_used,
                                    ground_spawned, window_spawned, current_time
                                )
                            break
                        else:
                            time_window_drop[wi] = 0.0

                    if not matched:
                        time_window_drop[:] = 0.0

                        # Ground pads -- the drone can fill a ground slot
                        # too, it just earns no window bonus for it.
                        for bi, spec in enumerate(BUILDINGS):
                            if spec["ground"] is None:
                                continue
                            dx = loc_drone[0] - spec["ground"][0]
                            dy = loc_drone[1] - spec["ground"][1]
                            dz = loc_drone[2] - spec["ground"][2]

                            if (
                                np.hypot(dx, dy) < 0.5
                                and 0.0 <= dz <= 4.0
                                and building_filled[bi] < spec["slots"]
                            ):
                                time_drone_ground[bi] += dt
                                print(f"QDrone2 ground drop-off timer [{spec['name']}]: "
                                      f"{time_drone_ground[bi]:.2f}")
                                matched = True

                                if time_drone_ground[bi] >= DURATION_DROP:
                                    building_filled[bi] += 1
                                    score += delivery_score(current_time)
                                    show_score_time(qlabs_sys, score, current_time)
                                    print(f"{spec['name']}: ground delivery "
                                          f"{building_filled[bi]}/{spec['slots']}")
                                    no_small_box_drone = 0
                                    destroy_box(qlabs_basic_shape, actor_number=103)
                                    time_drone_ground[bi] = 0.0

                                    reveal_building(
                                        qlabs_basic_shape, bi, building_filled, window_used,
                                        ground_spawned, window_spawned, current_time
                                    )
                                break
                            else:
                                time_drone_ground[bi] = 0.0

                        if not matched:
                            time_drone_ground[:] = 0.0
                else:
                    time_window_drop[:] = 0.0
                    time_drone_ground[:] = 0.0

            else:
                time_drone_pick_small = 0.0
                if intention_drone != 2:
                    time_window_drop[:] = 0.0
                    time_drone_ground[:] = 0.0

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
            # End of run

            if current_time >= SCORE_TIME_CAP:
                for _b in range(len(BUILDINGS)):
                    reveal_building(
                        qlabs_basic_shape, _b, building_filled, window_used,
                        ground_spawned, window_spawned, current_time
                    )
                show_score_time_game_over(
                    qlabs_sys, score - PENALTY_PER_HIT * curb_hits, current_time
                )
                end_reason = f"{SCORE_TIME_CAP / 60:.0f} minutes up"
                end_time = current_time
                break

            # Check if all deliveries are completed
            if np.sum(building_filled) + np.sum(completed_large) == TOTAL_DELIVERIES:
                show_score_time_game_over(
                    qlabs_sys, score - PENALTY_PER_HIT * curb_hits, current_time
                )
                end_reason = "all deliveries completed"
                end_time = current_time
                break

            timer.sleep()

    finally:
        curb_penalty = PENALTY_PER_HIT * curb_hits
        slots_done = int(np.sum(building_filled))
        large_done = int(np.sum(completed_large))
        total_done = slots_done + large_done

        window_done = int(np.sum(window_used))
        common_done = slots_done - window_done
        mins, secs = int(end_time) // 60, int(end_time) % 60

        print()
        print("=" * 48)
        print("                     RESULT")
        print("=" * 48)
        print(f"  {'End of run':<22}: {end_reason} at {mins:d}:{secs:02d}")
        print(f"  {'Deliveries completed':<22}: {total_done:>6} / {TOTAL_DELIVERIES}")
        print(f"  {'   Shared drop-off':<22}: {common_done:>6}")
        print(f"  {'   Window delivery':<22}: {window_done:>6}")
        print(f"  {'   Large delivery':<22}: {large_done:>6} / {len(LOC_LARGE_DROPOFF_LIST)}")
        print(f"  {'Curb hits':<22}: {curb_hits:>6}")
        print("  " + "-" * 44)
        print(f"  {'Score':<22}: {score:>10.0f}")
        print(f"  {'Penalty':<22}: {-curb_penalty:>10.0f}   ({curb_hits} x {PENALTY_PER_HIT:.0f})")
        print(f"  {'Final score':<22}: {score - curb_penalty:>10.0f}")
        print("=" * 48)

        if curb_log:
            print("\n  Co-ordinates where it hit:")
            for n, (t_hit, hx, hy, why) in enumerate(curb_log, 1):
                print(f"    {n:>2}   ({hx:7.2f}, {hy:7.2f})")

        if curb_TRACE and curb_trace:
            try:
                trace_path = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), "curb_trace.csv")
                with open(trace_path, "w") as fh:
                    fh.write("t,x,y,yaw,wheels_off,depth\n")
                    for t_r, x_r, y_r, yaw_r, off_r, d_r in curb_trace:
                        fh.write(f"{t_r:.2f},{x_r:.4f},{y_r:.4f},"
                                 f"{yaw_r:.5f},{off_r},{d_r:.3f}\n")
                print("\n  trace written : curb_trace.csv")
            except Exception as exc:
                print(f"\n  could not write the trace: {exc}")

        if not grid.ok:
            print("\n  NOTE: no drivable grid was loaded, so no curb test ran."
                  " Run with 'drivable_grid.npz' in the same folder as"
                  " 'game_finals.py'.")
        else:
            print("\n  All files present. Run plot_trace.py to see where"
                  " hits were recorded.")
        print()

        server_car.terminate()
        server_drone.terminate()
        qlabs.close()


if __name__ == "__main__":
    os.system("cls")
    print("=" * 60)
    print("game_finals.py ")
    print("=" * 60)
    print()

    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        try:
            input("Press Enter to close this window...")
        except EOFError:
            pass
