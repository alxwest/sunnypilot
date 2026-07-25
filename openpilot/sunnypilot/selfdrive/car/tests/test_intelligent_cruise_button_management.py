from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def _icbm() -> IntelligentCruiseButtonManagement:
  return IntelligentCruiseButtonManagement(SimpleNamespace(), SimpleNamespace(pcmCruiseSpeed=False))


def _car_state(v_ego_mph: float, set_speed_mph: float):
  return SimpleNamespace(
    vEgo=v_ego_mph * CV.MPH_TO_MS,
    cruiseState=SimpleNamespace(speedCluster=set_speed_mph * CV.MPH_TO_MS),
    buttonEvents=[],
  )


def _longitudinal_plan(speed_limit_mph: float = 40.0):
  resolver = SimpleNamespace(
    source=SpeedLimitSource.car,
    speedLimitValid=True,
    speedLimit=speed_limit_mph * CV.MPH_TO_MS,
    speedLimitFinal=speed_limit_mph * CV.MPH_TO_MS,
  )
  return SimpleNamespace(
    speedLimit=SimpleNamespace(resolver=resolver),
    longitudinalPlanSource=LongitudinalPlanSource.cruise,
    vTarget=speed_limit_mph * CV.MPH_TO_MS,
  )


def _live_map_data(speed_limit_ahead_mph: float, distance_m: float, valid: bool = True):
  return SimpleNamespace(
    speedLimitAheadValid=valid,
    speedLimitAhead=speed_limit_ahead_mph * CV.MPH_TO_MS,
    speedLimitAheadDistance=distance_m,
  )


def test_map_speed_limit_ahead_lowers_icbm_target_when_within_stock_acc_distance():
  icbm = _icbm()
  CS = _car_state(v_ego_mph=40.0, set_speed_mph=40.0)

  icbm.is_metric = False
  icbm.update_calculations(CS, _longitudinal_plan(40.0), _live_map_data(30.0, 120.0))

  assert icbm.set_speed_target == 30
  assert icbm.v_target == 30


def test_map_speed_limit_ahead_waits_when_too_far_away():
  icbm = _icbm()
  CS = _car_state(v_ego_mph=40.0, set_speed_mph=40.0)

  icbm.is_metric = False
  icbm.update_calculations(CS, _longitudinal_plan(40.0), _live_map_data(30.0, 400.0))

  assert icbm.set_speed_target == 40
  assert icbm.v_target == 40
