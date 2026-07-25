import json
import math
import platform

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.navd.helpers import coordinate_from_param, Coordinate
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState

ACTIVE_STATES = (MapState.turning, )
ENABLED_STATES = (MapState.enabled, MapState.overriding, *ACTIVE_STATES)

R = 6373000.0  # approximate radius of earth in meters
TO_RADIANS = math.pi / 180
TO_DEGREES = 180 / math.pi
TARGET_JERK = -0.6  # m/s^3 There's some jounce limits that are not consistent so we're fudging this some
TARGET_ACCEL = -1.2  # m/s^2 should match up with the long planner limit
TARGET_OFFSET = 1.0  # seconds - This controls how soon before the curve you reach the target velocity. It also helps
                     # reach the target velocity when inaccuracies in the distance modeling logic would cause overshoot.
                     # The value is multiplied against the target velocity to determine the additional distance. This is
                     # done to keep the distance calculations consistent but results in the offset actually being less
                     # time than specified depending on how much of a speed differential there is between v_ego and the
                     # target velocity.
STOCK_ACC_DECEL = 0.6  # m/s^2, conservative assumed stock ACC decel for set-speed-only curve control.
STOCK_ACC_RESPONSE_TIME = 5.0  # seconds, accounts for virtual button and stock ACC response delay.
CURVE_DISTANCE_BUFFER = 35.0  # meters, extra buffer before the estimated decel point.
CURVE_EXIT_HOLD_TIME = 1.0  # seconds, keeps the low set-speed target briefly after map confidence drops.
CURVE_EXIT_HOLD_FRAMES = int(CURVE_EXIT_HOLD_TIME / DT_MDL)
CURVE_RELEASE_SPEED_MARGIN = 1.8  # m/s, about 4 mph. Allows acceleration once the target speed has been reached.
OUTSIDE_CURVE_SPEED_FACTOR = 1.08
MAX_OUTSIDE_CURVE_SPEED_BONUS = 2.0  # m/s, about 4.5 mph.
MIN_CURVE_DIRECTION_DISTANCE = 10.0
MIN_CURVE_DIRECTION_SIN = 0.03
LEFT_HAND_TRAFFIC_COUNTRIES = frozenset({
  "AG", "AI", "AU", "BB", "BD", "BM", "BN", "BS", "BT", "BW", "CK", "CY", "DM", "FJ", "FK", "GB", "GD", "GG",
  "GY", "HK", "ID", "IE", "IM", "IN", "JE", "JM", "JP", "KE", "KI", "KN", "KY", "LC", "LK", "LS", "MO", "MS",
  "MT", "MU", "MV", "MW", "MY", "MZ", "NA", "NP", "NR", "NU", "NZ", "PG", "PK", "PN", "SB", "SC", "SG", "SH",
  "SR", "SZ", "TC", "TH", "TL", "TO", "TT", "TV", "TZ", "UG", "UK", "VC", "VG", "VI", "WS", "ZA", "ZM", "ZW",
})
LEFT_HAND_TRAFFIC_COUNTRY_NAMES = frozenset({
  "united kingdom",
  "great britain",
  "england",
  "scotland",
  "wales",
  "northern ireland",
  "ireland",
  "japan",
  "australia",
  "new zealand",
})


def velocities_from_param(param: str, params: Params):
  if params is None:
    params = Params()

  json_str = params.get(param)
  if json_str is None:
    return None

  velocities = json.loads(json_str)

  return velocities


def calculate_accel(t, target_jerk, a_ego):
  return a_ego + target_jerk * t


def calculate_velocity(t, target_jerk, a_ego, v_ego):
  return v_ego + a_ego * t + target_jerk/2 * (t ** 2)


def calculate_distance(t, target_jerk, a_ego, v_ego):
  return t * v_ego + a_ego/2 * (t ** 2) + target_jerk/6 * (t ** 3)


# points should be in radians
# output is meters
def distance_to_point(ax, ay, bx, by):
  a = math.sin((bx-ax)/2)*math.sin((bx-ax)/2) + math.cos(ax) * math.cos(bx)*math.sin((by-ay)/2)*math.sin((by-ay)/2)
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

  return R * c  # in meters


class SmartCruiseControlMap:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.enabled = self.params.get_bool("SmartCruiseControlMap")
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.state = MapState.disabled
    self.v_cruise = 0
    self.target_lat = 0.0
    self.target_lon = 0.0
    self.target_map_velocity = 0.0
    self.curve_hold_frames = 0
    self.left_hand_traffic = False
    self.left_hand_traffic_fallback = False
    self.country_left_hand_traffic: bool | None = None
    self.frame = -1

    self.last_position = coordinate_from_param("LastGPSPosition", self.mem_params) or Coordinate(0.0, 0.0)
    self.target_velocities = velocities_from_param("MapTargetVelocities", self.mem_params) or []
    self._update_country_traffic_side()
    self._update_traffic_side()

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V)

    return V_CRUISE_UNSET

  def get_a_target_from_control(self) -> float:
    return self.a_ego

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlMap")
      self._update_country_traffic_side()

  @staticmethod
  def _normalize_country(country: str | None) -> str:
    return (country or "").strip().upper().replace("-", "_")

  @classmethod
  def _country_uses_left_hand_traffic(cls, country: str | None) -> bool | None:
    normalized = cls._normalize_country(country)
    if not normalized:
      return None
    if normalized in LEFT_HAND_TRAFFIC_COUNTRIES:
      return True
    if normalized.lower() in LEFT_HAND_TRAFFIC_COUNTRY_NAMES:
      return True
    return False

  def _update_country_traffic_side(self) -> None:
    country = self.params.get("OsmLocationName", return_default=True)
    self.country_left_hand_traffic = self._country_uses_left_hand_traffic(country)

  def _update_traffic_side(self) -> None:
    self.left_hand_traffic = (
      self.left_hand_traffic_fallback if self.country_left_hand_traffic is None else self.country_left_hand_traffic
    )

  def _target_still_ahead(self, forward_points: list[dict]) -> bool:
    if self.v_target <= 0. or self.target_lat == 0. or self.target_lon == 0.:
      return False

    for target_velocity in forward_points:
      target_map_velocity = self.target_map_velocity if self.target_map_velocity > 0. else self.v_target
      if (
        target_velocity["latitude"] == self.target_lat and
        target_velocity["longitude"] == self.target_lon and
        target_velocity["velocity"] == target_map_velocity
      ):
        return True

    return False

  def _reset_target(self) -> None:
    self.v_target = 0.0
    self.target_lat = 0.0
    self.target_lon = 0.0
    self.target_map_velocity = 0.0
    self.curve_hold_frames = 0

  @staticmethod
  def _target_point_distance(a: dict, b: dict) -> float:
    return distance_to_point(
      a["latitude"] * TO_RADIANS, a["longitude"] * TO_RADIANS,
      b["latitude"] * TO_RADIANS, b["longitude"] * TO_RADIANS,
    )

  @staticmethod
  def _target_point_xy(point: dict, origin: dict) -> tuple[float, float]:
    origin_lat_rad = origin["latitude"] * TO_RADIANS
    x = (point["longitude"] - origin["longitude"]) * TO_RADIANS * R * math.cos(origin_lat_rad)
    y = (point["latitude"] - origin["latitude"]) * TO_RADIANS * R
    return x, y

  def _curve_reference_index(self, points: list[dict], start_idx: int, step: int) -> int | None:
    idx = start_idx
    while 0 <= idx + step < len(points):
      idx += step
      if self._target_point_distance(points[start_idx], points[idx]) >= MIN_CURVE_DIRECTION_DISTANCE:
        return idx

    return idx if idx != start_idx else None

  def _signed_curve_turn(self, forward_points: list[dict], idx: int) -> float:
    if not forward_points:
      return 0.0

    current_position = {"latitude": self.last_position.latitude, "longitude": self.last_position.longitude}
    points = [current_position, *forward_points]
    center_idx = idx + 1
    if center_idx >= len(points):
      return 0.0

    prev_idx = self._curve_reference_index(points, center_idx, -1)
    next_idx = self._curve_reference_index(points, center_idx, 1)
    if prev_idx is None or next_idx is None:
      return 0.0

    center = points[center_idx]
    p0x, p0y = self._target_point_xy(points[prev_idx], center)
    p2x, p2y = self._target_point_xy(points[next_idx], center)
    v1x, v1y = -p0x, -p0y
    v2x, v2y = p2x, p2y
    mag1 = math.hypot(v1x, v1y)
    mag2 = math.hypot(v2x, v2y)
    if mag1 < 1.0 or mag2 < 1.0:
      return 0.0

    return (v1x * v2y - v1y * v2x) / (mag1 * mag2)

  def _target_velocity_for_lane_side(self, target_velocity: float, forward_points: list[dict], idx: int) -> float:
    signed_turn = self._signed_curve_turn(forward_points, idx)
    if abs(signed_turn) < MIN_CURVE_DIRECTION_SIN:
      return target_velocity

    turn_is_right = signed_turn < 0.
    outside_curve = turn_is_right if self.left_hand_traffic else not turn_is_right
    if not outside_curve:
      return target_velocity

    speed_bonus = min(
      target_velocity * (OUTSIDE_CURVE_SPEED_FACTOR - 1.),
      MAX_OUTSIDE_CURVE_SPEED_BONUS,
    )
    adjusted_target = target_velocity + speed_bonus
    return min(adjusted_target, self.v_cruise) if self.v_cruise > 0. else adjusted_target

  def update_calculations(self) -> None:
    self.last_position = coordinate_from_param("LastGPSPosition", self.mem_params) or Coordinate(0.0, 0.0)
    lat = self.last_position.latitude
    lon = self.last_position.longitude

    self.target_velocities = velocities_from_param("MapTargetVelocities", self.mem_params) or []

    if self.last_position is None or self.target_velocities is None:
      return

    min_dist = 1000
    min_idx = 0
    distances = []

    # find our location in the path
    for i in range(len(self.target_velocities)):
      target_velocity = self.target_velocities[i]
      tlat = target_velocity["latitude"]
      tlon = target_velocity["longitude"]
      d = distance_to_point(lat * TO_RADIANS, lon * TO_RADIANS, tlat * TO_RADIANS, tlon * TO_RADIANS)
      distances.append(d)
      if d < min_dist:
        min_dist = d
        min_idx = i

    # only look at values from our current position forward
    forward_points = self.target_velocities[min_idx:]
    forward_distances = distances[min_idx:]

    # find velocities that we are within the distance we need to adjust for
    valid_velocities = []
    recovering_from_curve = self.v_target > 0. and self.v_ego <= self.v_target + CURVE_RELEASE_SPEED_MARGIN
    for i in range(len(forward_points)):
      target_velocity = forward_points[i]
      tlat = target_velocity["latitude"]
      tlon = target_velocity["longitude"]
      map_tv = target_velocity["velocity"]
      tv = self._target_velocity_for_lane_side(map_tv, forward_points, i)
      if tv > self.v_ego and not (recovering_from_curve and tv < self.v_cruise):
        continue

      d = forward_distances[i]

      a_diff = (self.a_ego - TARGET_ACCEL)
      accel_t = abs(a_diff / TARGET_JERK)
      min_accel_v = calculate_velocity(accel_t, TARGET_JERK, self.a_ego, self.v_ego)

      max_d = 0
      if tv > self.v_ego:
        max_d = 0
      elif tv > min_accel_v:
        # calculate time needed based on target jerk
        a = 0.5 * TARGET_JERK
        b = self.a_ego
        c = self.v_ego - tv
        t_a = -1 * ((b**2 - 4 * a * c) ** 0.5 + b) / 2 * a
        t_b = ((b**2 - 4 * a * c) ** 0.5 - b) / 2 * a
        if not isinstance(t_a, complex) and t_a > 0:
          t = t_a
        else:
          t = t_b
        if isinstance(t, complex):
          continue

        max_d = max_d + calculate_distance(t, TARGET_JERK, self.a_ego, self.v_ego)
      else:
        t = accel_t
        max_d = calculate_distance(t, TARGET_JERK, self.a_ego, self.v_ego)

        # calculate additional time needed based on target accel
        t = abs((min_accel_v - tv) / TARGET_ACCEL)
        max_d += calculate_distance(t, 0, TARGET_ACCEL, min_accel_v)

      stock_acc_decel_distance = max(0., self.v_ego ** 2 - tv ** 2) / (2. * STOCK_ACC_DECEL)
      response_distance = self.v_ego * STOCK_ACC_RESPONSE_TIME
      anticipation_distance = (
        max(max_d, stock_acc_decel_distance) + response_distance + CURVE_DISTANCE_BUFFER + tv * TARGET_OFFSET
      )

      if d < anticipation_distance:
        valid_velocities.append((float(tv), tlat, tlon, float(map_tv)))

    # Find the smallest velocity we need to adjust for
    min_v = 100.0
    target_lat = 0.0
    target_lon = 0.0
    target_map_velocity = 0.0
    for tv, lat, lon, map_tv in valid_velocities:
      if tv < min_v:
        min_v = tv
        target_lat = lat
        target_lon = lon
        target_map_velocity = map_tv

    has_new_target = min_v < 100.0
    previous_target_still_ahead = self._target_still_ahead(forward_points)
    near_target_speed = self.v_target > 0. and self.v_ego <= self.v_target + CURVE_RELEASE_SPEED_MARGIN

    # Keep a lower active target until the map point has actually passed, even if a later point is less restrictive.
    if previous_target_still_ahead and not near_target_speed and (not has_new_target or self.v_target < min_v):
      self.curve_hold_frames = CURVE_EXIT_HOLD_FRAMES
      return

    if has_new_target:
      self.v_target = min_v
      self.target_lat = target_lat
      self.target_lon = target_lon
      self.target_map_velocity = target_map_velocity
      self.curve_hold_frames = CURVE_EXIT_HOLD_FRAMES
      return

    # Mapd can briefly drop target velocity points at intersections or road-name changes. Hold the previous target
    # for a short grace period so ICBM does not immediately raise the stock ACC set speed mid-corner.
    if self.v_target > 0. and self.curve_hold_frames > 0:
      self.curve_hold_frames -= 1
      return

    self._reset_target()

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, TURNING
    if self.state != MapState.disabled:
      if not self.long_enabled or not self.enabled:
        self.state = MapState.disabled
      elif self.long_override:
        self.state = MapState.overriding

      else:
        # ENABLED
        if self.state == MapState.enabled:
          if self.v_cruise > self.v_target != 0 and self.v_ego > self.v_target + CURVE_RELEASE_SPEED_MARGIN:
            self.state = MapState.turning

        # TURNING
        elif self.state == MapState.turning:
          if self.v_target == 0 or self.v_ego <= self.v_target + CURVE_RELEASE_SPEED_MARGIN:
            self.state = MapState.enabled

        # OVERRIDING
        elif self.state == MapState.overriding:
          if not self.long_override:
            if self.v_cruise > self.v_target != 0 and self.v_ego > self.v_target + CURVE_RELEASE_SPEED_MARGIN:
              self.state = MapState.turning
            else:
              self.state = MapState.enabled

    # DISABLED
    elif self.state == MapState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = MapState.overriding
        else:
          self.state = MapState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def update(self, long_enabled: bool, long_override: bool, v_ego, a_ego, v_cruise,
             left_hand_traffic_fallback: bool = False) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise = v_cruise
    self.left_hand_traffic_fallback = left_hand_traffic_fallback

    self.update_params()
    self._update_traffic_side()
    self.update_calculations()

    self.is_enabled, self.is_active = self._update_state_machine()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
