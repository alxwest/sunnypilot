from __future__ import annotations

import os
from pathlib import Path

from openpilot.common.hardware import PC
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.hardware.hw import Paths

PARKING_ENABLED_PARAM = "ParkingMotionRecording"
PARKING_ROUTE_COUNT_PARAM = "ParkingRouteCount"
CUSTOM_PARAM_TYPES = {
  PARKING_ENABLED_PARAM: "bool",
  PARKING_ROUTE_COUNT_PARAM: "int",
}


def parse_bool(value: bytes) -> bool:
  normalized = value.strip().lower()
  if normalized in (b"1", b"true"):
    return True
  if normalized in (b"0", b"false"):
    return False
  raise ValueError(f"Invalid boolean value: {value!r}")


def _is_registered(params: Params, key: str) -> bool:
  try:
    params.check_key(key)
    return True
  except UnknownKeyName:
    return False


def _fallback_path(key: str) -> Path:
  root = Path(Paths.comma_home()) if PC else Path("/data")
  return root / "parking_recorder" / key


def _read_fallback(key: str) -> str | None:
  try:
    return _fallback_path(key).read_text()
  except OSError:
    return None


def _write_fallback(key: str, value: str) -> None:
  path = _fallback_path(key)
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_suffix(".tmp")
  tmp.write_text(value)
  os.replace(tmp, path)


def get_bool(params: Params, key: str) -> bool:
  if _is_registered(params, key):
    return params.get_bool(key)
  return _read_fallback(key) == "1"


def put_bool(params: Params, key: str, value: bool) -> None:
  if _is_registered(params, key):
    params.put_bool(key, value, block=True)
  else:
    _write_fallback(key, "1" if value else "0")


def get_int(params: Params, key: str, default: int = 0) -> int:
  try:
    value = params.get(key) if _is_registered(params, key) else _read_fallback(key)
    return int(value) if value is not None else default
  except ValueError:
    return default


def put_int(params: Params, key: str, value: int) -> None:
  if _is_registered(params, key):
    params.put(key, value, block=True)
  else:
    _write_fallback(key, str(value))
