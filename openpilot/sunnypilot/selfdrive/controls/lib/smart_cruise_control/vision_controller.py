"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

_ENTERING_PRED_LAT_ACC_TH = 1.3  # Predicted Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # Predicted Lat Acc threshold to abort entering state if speed drops.

_TURNING_LAT_ACC_TH = 1.6  # Lat Acc threshold to trigger turning state.

_LEAVING_LAT_ACC_TH = 1.3  # Lat Acc threshold to trigger leaving turn state.
_FINISH_LAT_ACC_TH = 1.1  # Lat Acc threshold to trigger the end of the turn cycle.

_A_LAT_REG_MAX = 2.0  # Maximum lateral acceleration

_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# Lookup table for the minimum smooth deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]  # min decel value allowed on ENTERING state
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]  # absolute value of lat acc ahead

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.5, 0., -0.4]  # acc value
_TURNING_ACC_BP = [1.5, 2.3, 3.]  # absolute value of current lat acc

_LEAVING_ACC = 0.5  # Conformable acceleration to regain speed while leaving a turn.

_ANTICIPATION_PRED_LAT_ACC_TH = 1.1  # Predicted lat accel threshold used to find the start of an upcoming curve.
_STOCK_ACC_DECEL = 0.6  # m/s^2, conservative assumed stock ACC decel for set-speed-only curve control.
_STOCK_ACC_RESPONSE_TIME = 5.0  # seconds, accounts for virtual button and stock ACC response delay.
_CURVE_DISTANCE_BUFFER = 35.0  # meters, extra buffer before the estimated decel point.
_CURVE_EXIT_HOLD_TIME = 1.0  # seconds, keeps the low set-speed target briefly after confidence drops.
_CURVE_EXIT_HOLD_FRAMES = int(_CURVE_EXIT_HOLD_TIME / DT_MDL)
_MAX_EXTRA_CURVE_SET_SPEED_REDUCTION = 3.0  # m/s, cap extra temporary set-speed reduction to about 7 mph.


class SmartCruiseControlVision:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.
    self.distance_to_curve = float("inf")
    self.curve_decel_required = False
    self.curve_set_speed_target = V_CRUISE_UNSET
    self.curve_hold_frames = 0
    self.curve_hold_v_target = V_CRUISE_UNSET

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      v_target = max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON
      if self.curve_decel_required and self.state != VisionState.leaving:
        v_target = min(v_target, self.curve_set_speed_target)
      if self._curve_hold_active and self.state != VisionState.leaving:
        v_target = min(v_target, self.curve_hold_v_target)
      return v_target

    return V_CRUISE_UNSET

  @property
  def _curve_hold_active(self) -> bool:
    return self.curve_hold_frames > 0 and self.curve_hold_v_target != V_CRUISE_UNSET

  def _reset_curve_hold(self) -> None:
    self.curve_hold_frames = 0
    self.curve_hold_v_target = V_CRUISE_UNSET

  def _update_curve_hold(self) -> None:
    if self.state == VisionState.leaving:
      self._reset_curve_hold()
      return

    target_speed = max(self.v_target, MIN_V)
    if self.curve_decel_required:
      target_speed = min(target_speed, self.curve_set_speed_target)

    already_in_curve_state = self.state in ACTIVE_STATES
    curve_still_present = (
      self.curve_decel_required or
      self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH or
      (already_in_curve_state and self.current_lat_acc >= _FINISH_LAT_ACC_TH) or
      (already_in_curve_state and self.max_pred_lat_acc >= _ABORT_ENTERING_PRED_LAT_ACC_TH)
    )

    if curve_still_present and target_speed < V_CRUISE_UNSET:
      self.curve_hold_frames = _CURVE_EXIT_HOLD_FRAMES
      if self.curve_hold_v_target == V_CRUISE_UNSET:
        self.curve_hold_v_target = target_speed
      else:
        self.curve_hold_v_target = min(self.curve_hold_v_target, target_speed)
    elif self.curve_hold_frames > 0:
      self.curve_hold_frames -= 1
    else:
      self.curve_hold_v_target = V_CRUISE_UNSET

    if self._curve_hold_active:
      self.v_target = min(self.v_target, self.curve_hold_v_target)
      self.curve_set_speed_target = min(self.curve_set_speed_target, self.curve_hold_v_target)

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      self._reset_curve_hold()
      return
    else:
      rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
      vel_plan = np.array(sm['modelV2'].velocity.x)
      position_plan = np.array(sm['modelV2'].position.x)
      plan_len = min(len(rate_plan), len(vel_plan), len(position_plan))

      if plan_len == 0:
        self.max_pred_lat_acc = 0.
        self.distance_to_curve = float("inf")
        self.curve_decel_required = False
        if self._curve_hold_active:
          self._update_curve_hold()
        else:
          self.curve_set_speed_target = V_CRUISE_UNSET
        return

      rate_plan = rate_plan[:plan_len]
      vel_plan = vel_plan[:plan_len]
      position_plan = position_plan[:plan_len]

      self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

      # get the maximum lat accel from the model
      predicted_lat_accels = rate_plan * vel_plan
      self.max_pred_lat_acc = np.percentile(predicted_lat_accels, 97)
      if self.max_pred_lat_acc <= 0.:
        self.v_target = self.curve_hold_v_target if self._curve_hold_active else self.v_cruise_setpoint
        self.distance_to_curve = float("inf")
        self.curve_decel_required = False
        if self._curve_hold_active:
          self.curve_set_speed_target = self.curve_hold_v_target
          self._update_curve_hold()
        else:
          self.curve_set_speed_target = V_CRUISE_UNSET
        return

      # get the maximum curve based on the current velocity
      v_ego = max(self.v_ego, 0.1)  # ensure a value greater than 0 for calculations
      max_curve = self.max_pred_lat_acc / (v_ego**2)

      # Get the target velocity for the maximum curve
      self.v_target = (_A_LAT_REG_MAX / max_curve) ** 0.5

      curve_indices = (
        np.flatnonzero(predicted_lat_accels >= _ANTICIPATION_PRED_LAT_ACC_TH)
        if self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH else []
      )
      self.distance_to_curve = float(position_plan[curve_indices[0]]) if len(curve_indices) else float("inf")
      target_speed = max(self.v_target, MIN_V)
      decel_distance = max(0., self.v_ego ** 2 - target_speed ** 2) / (2. * _STOCK_ACC_DECEL)
      response_distance = self.v_ego * _STOCK_ACC_RESPONSE_TIME
      self.curve_decel_required = (
        self.v_ego > target_speed and
        self.distance_to_curve <= decel_distance + response_distance + _CURVE_DISTANCE_BUFFER
      )
      usable_distance = max(1., self.distance_to_curve - response_distance - _CURVE_DISTANCE_BUFFER)
      required_decel = max(0., self.v_ego ** 2 - target_speed ** 2) / (2. * usable_distance)
      extra_reduction = np.clip(
        (required_decel - _STOCK_ACC_DECEL) * _STOCK_ACC_RESPONSE_TIME,
        0.,
        _MAX_EXTRA_CURVE_SET_SPEED_REDUCTION,
      )
      self.curve_set_speed_target = max(MIN_V, target_speed - extra_reduction)
      self._update_curve_hold()

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # Do not enter a turn control cycle if the speed is low.
          if self.v_ego <= MIN_V:
            pass
          # If significant lateral acceleration is predicted ahead, then move to Entering turn state.
          elif self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH or self.curve_decel_required or self._curve_hold_active:
            self.state = VisionState.entering

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          # Transition to Turning if current lateral acceleration is over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Abort if the predicted lateral acceleration drops
          elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH and not self.curve_decel_required and not self._curve_hold_active:
            self.state = VisionState.enabled

        # TURNING
        elif self.state == VisionState.turning:
          # Transition to Leaving if current lateral acceleration drops below a threshold.
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self._reset_curve_hold()
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          # Transition back to Turning if current lateral acceleration goes back over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Finish if current lateral acceleration goes below a threshold.
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH:
            self.state = VisionState.enabled

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_solution(self) -> float:
    # DISABLED, ENABLED, OVERRIDING
    if self.state not in ACTIVE_STATES:
      # when not overshooting, calculate v_turn as the speed at the prediction horizon when following
      # the smooth deceleration.
      a_target = self.a_ego
    # ENTERING
    elif self.state == VisionState.entering:
      # when not overshooting, target a smooth deceleration in preparation for a sharp turn to come.
      a_target = np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
    # TURNING
    elif self.state == VisionState.turning:
      # When turning, we provide a target acceleration that is comfortable for the lateral acceleration felt.
      a_target = np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING
    elif self.state == VisionState.leaving:
      # When leaving, we provide a comfortable acceleration to regain speed.
      a_target = _LEAVING_ACC
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
