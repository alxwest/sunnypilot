"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4
V_TARGET_UNSET = 0.0
CURVE_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap)
STOCK_ACC_SPEED_LIMIT_DECEL = 0.6  # m/s^2, conservative stock ACC response for virtual-button speed sync.
STOCK_ACC_SPEED_LIMIT_RESPONSE_TIME = 5.0
SPEED_LIMIT_AHEAD_DISTANCE_BUFFER = 35.0


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER
    self.set_speed_target = 0
    self.set_speed_target_last = 0
    self.set_speed_sync_active = False
    self.manual_button_pressed = False

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def _get_speed_limit_target(self, LP_SP: custom.LongitudinalPlanSP, speed_conv: float) -> int:
    resolver = LP_SP.speedLimit.resolver
    speed_limit = resolver.speedLimitFinal if resolver.speedLimitFinal > 0. else resolver.speedLimit
    speed_limit_valid = resolver.source != SpeedLimitSource.none and resolver.speedLimitValid and speed_limit > 0.

    if speed_limit_valid:
      return max(self.v_cruise_min, round(speed_limit * speed_conv))

    return 0

  def _get_map_speed_limit_ahead_target(self, CS: car.CarState, live_map_data_sp, speed_conv: float) -> int:
    if live_map_data_sp is None or not live_map_data_sp.speedLimitAheadValid or live_map_data_sp.speedLimitAhead <= 0.:
      return 0

    target = max(self.v_cruise_min, round(live_map_data_sp.speedLimitAhead * speed_conv))
    if target >= self.v_cruise_cluster:
      return 0

    approach_speed = max(CS.vEgo, self.v_cruise_cluster / speed_conv)
    decel_distance = max(0., approach_speed ** 2 - live_map_data_sp.speedLimitAhead ** 2) / (2. * STOCK_ACC_SPEED_LIMIT_DECEL)
    response_distance = approach_speed * STOCK_ACC_SPEED_LIMIT_RESPONSE_TIME
    anticipation_distance = decel_distance + response_distance + SPEED_LIMIT_AHEAD_DISTANCE_BUFFER

    if live_map_data_sp.speedLimitAheadDistance <= anticipation_distance:
      return target

    return 0

  def _get_curve_target(self, LP_SP: custom.LongitudinalPlanSP, speed_conv: float) -> int:
    if LP_SP.longitudinalPlanSource in CURVE_SOURCES and LP_SP.vTarget > 0.:
      return max(self.v_cruise_min, round(LP_SP.vTarget * speed_conv))

    return 0

  def update_set_speed_sync(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP, live_map_data_sp, speed_conv: float) -> float:
    speed_limit_target = self._get_speed_limit_target(LP_SP, speed_conv)
    map_speed_limit_ahead_target = self._get_map_speed_limit_ahead_target(CS, live_map_data_sp, speed_conv)
    curve_target = self._get_curve_target(LP_SP, speed_conv)

    if speed_limit_target > 0 or map_speed_limit_ahead_target > 0 or curve_target > 0:
      targets = [target for target in (speed_limit_target, map_speed_limit_ahead_target, curve_target) if target > 0]
      self.set_speed_target = min(targets)
    elif LP_SP.vTarget > 0.:
      self.set_speed_target = max(self.v_cruise_min, round(LP_SP.vTarget * speed_conv))
    else:
      self.set_speed_target = 0
      self.set_speed_target_last = 0
      self.set_speed_sync_active = False
      return V_TARGET_UNSET

    if self.manual_button_pressed:
      self.set_speed_sync_active = False
    elif self.set_speed_target != self.set_speed_target_last:
      self.set_speed_sync_active = self.set_speed_target != self.v_cruise_cluster
    elif self.set_speed_sync_active and self.set_speed_target == self.v_cruise_cluster:
      self.set_speed_sync_active = False

    self.set_speed_target_last = self.set_speed_target
    if self.set_speed_sync_active:
      return self.set_speed_target / speed_conv

    return self.v_cruise_cluster / speed_conv

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP, live_map_data_sp=None) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

    v_target = self.update_set_speed_sync(CS, LP_SP, live_map_data_sp, speed_conv)
    if v_target == V_TARGET_UNSET:
      v_target = LP_SP.vTarget

    self.v_target_ms_last = apply_hysteresis(v_target, self.v_target_ms_last, HYST_GAP * ms_conv)
    self.v_target = round(self.v_target_ms_last * speed_conv)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster:
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    self.manual_button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    self.is_ready = ready and not self.manual_button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool,
          live_map_data_sp=None) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_readiness(CS, CC)
    self.update_calculations(CS, LP_SP, live_map_data_sp)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
