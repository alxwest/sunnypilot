import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
MRR35_RADAR_ADDR = 0x3A5
MRR35_RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_config(CP) -> tuple[int, int, int]:
  if CP.flags & HyundaiFlags.MRR35_RADAR:
    return MRR35_RADAR_ADDR, MRR35_RADAR_MSG_COUNT, CanBus(CP).ACAN
  return RADAR_START_ADDR, RADAR_MSG_COUNT, 1


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  radar_start_addr, radar_msg_count, radar_bus = get_radar_config(CP)
  frequency = float('nan') if CP.flags & HyundaiFlags.MRR35_RADAR else 50
  messages = [(f"RADAR_TRACK_{addr:x}", frequency) for addr in range(radar_start_addr, radar_start_addr + radar_msg_count)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, radar_bus)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.radar_start_addr, self.radar_msg_count, _ = get_radar_config(CP)
    self.updated_messages = set()
    self.trigger_msg = self.radar_start_addr if CP.flags & HyundaiFlags.MRR35_RADAR else self.radar_start_addr + self.radar_msg_count - 1

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        if self.CP.flags & HyundaiFlags.MRR35_RADAR:
          self.pts[addr].dRel = msg['LONG_DIST']
          self.pts[addr].yRel = msg['LAT_DIST']
        else:
          azimuth = math.radians(msg['AZIMUTH'])
          self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
          self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
