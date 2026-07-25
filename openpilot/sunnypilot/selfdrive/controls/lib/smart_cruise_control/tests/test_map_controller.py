"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import platform

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.navd.helpers import Coordinate
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState


class TestSmartCruiseControlMap:

  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.reset_params()
    self.scc_m = SmartCruiseControlMap()

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlMap", True, block=True)
    self.params.remove("OsmLocationName")

    # TODO-SP: mock data from gpsLocation
    self.params.put("LastGPSPosition", "{}", block=True)
    self.params.put("MapTargetVelocities", "{}", block=True)

  def test_initial_state(self):
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active
    assert self.scc_m.output_v_target == V_CRUISE_UNSET
    assert self.scc_m.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlMap", False, block=True)
    self.scc_m.enabled = self.params.get_bool("SmartCruiseControlMap")

    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(False, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.enabled

  def test_enabled_does_not_enter_curve_when_speed_is_near_target(self):
    self.scc_m.state = MapState.enabled
    self.scc_m.long_enabled = True
    self.scc_m.enabled = True
    self.scc_m.long_override = False
    self.scc_m.v_target = 20.0
    self.scc_m.v_ego = 21.0
    self.scc_m.v_cruise = 30.0

    self.scc_m._update_state_machine()

    assert self.scc_m.state == MapState.enabled

  def test_turning_exits_when_speed_has_reached_curve_target(self):
    self.scc_m.state = MapState.turning
    self.scc_m.long_enabled = True
    self.scc_m.enabled = True
    self.scc_m.long_override = False
    self.scc_m.v_target = 20.0
    self.scc_m.v_ego = 21.0
    self.scc_m.v_cruise = 30.0

    self.scc_m._update_state_machine()

    assert self.scc_m.state == MapState.enabled

  def test_turning_stays_active_when_speed_is_above_curve_target(self):
    self.scc_m.state = MapState.turning
    self.scc_m.long_enabled = True
    self.scc_m.enabled = True
    self.scc_m.long_override = False
    self.scc_m.v_target = 20.0
    self.scc_m.v_ego = 22.0
    self.scc_m.v_cruise = 30.0

    self.scc_m._update_state_machine()

    assert self.scc_m.state == MapState.turning

  def test_right_curve_target_is_higher_for_left_hand_traffic(self):
    self.scc_m.left_hand_traffic = True
    self.scc_m.last_position = Coordinate(0.0, 0.0)
    self.scc_m.v_cruise = 30.0
    forward_points = [
      {"latitude": 0.0010, "longitude": 0.0000, "velocity": 20.0},
      {"latitude": 0.0015, "longitude": 0.0005, "velocity": 20.0},
    ]

    adjusted = self.scc_m._target_velocity_for_lane_side(20.0, forward_points, 0)

    assert adjusted > 20.0

  def test_left_curve_target_is_not_higher_for_left_hand_traffic(self):
    self.scc_m.left_hand_traffic = True
    self.scc_m.last_position = Coordinate(0.0, 0.0)
    self.scc_m.v_cruise = 30.0
    forward_points = [
      {"latitude": 0.0010, "longitude": 0.0000, "velocity": 20.0},
      {"latitude": 0.0015, "longitude": -0.0005, "velocity": 20.0},
    ]

    adjusted = self.scc_m._target_velocity_for_lane_side(20.0, forward_points, 0)

    assert adjusted == 20.0

  def test_target_still_ahead_uses_raw_map_velocity_for_lane_adjusted_target(self):
    self.scc_m.v_target = 21.0
    self.scc_m.target_lat = 0.0010
    self.scc_m.target_lon = 0.0001
    self.scc_m.target_map_velocity = 20.0
    forward_points = [
      {"latitude": 0.0010, "longitude": 0.0001, "velocity": 20.0},
    ]

    assert self.scc_m._target_still_ahead(forward_points)

  def test_osm_country_sets_left_hand_traffic(self):
    self.params.put("OsmLocationName", "GB", block=True)

    self.scc_m._update_country_traffic_side()
    self.scc_m._update_traffic_side()

    assert self.scc_m.left_hand_traffic

  def test_osm_country_overrides_rhd_fallback(self):
    self.params.put("OsmLocationName", "FR", block=True)
    self.scc_m.left_hand_traffic_fallback = True

    self.scc_m._update_country_traffic_side()
    self.scc_m._update_traffic_side()

    assert not self.scc_m.left_hand_traffic

  def test_rhd_fallback_used_when_osm_country_is_unset(self):
    self.params.remove("OsmLocationName")
    self.scc_m.left_hand_traffic_fallback = True

    self.scc_m._update_country_traffic_side()
    self.scc_m._update_traffic_side()

    assert self.scc_m.left_hand_traffic

  # TODO-SP: mock data from modelV2 to test other states
