"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl


LEAD_TRACK_COLOR = rl.Color(0, 255, 64, 255)
RADAR_TRACK_COLOR = rl.Color(255, 105, 180, 255)


class RadarTracks:
  @staticmethod
  def _lead_track_id(radar_state) -> int | None:
    if radar_state is None or not radar_state.leadOne.present or not radar_state.leadOne.radar:
      return None

    track_id = radar_state.leadOne.radarTrackId
    return track_id if track_id >= 0 else None

  def draw_radar_tracks(self, live_tracks, radar_state, map_to_screen, path_offset_z, track_size=6):
    lead_track_id = self._lead_track_id(radar_state)

    for track in live_tracks.points:
      d_rel = track.dRel
      y_rel = track.yRel if math.isfinite(track.yRel) else 0.0
      if not math.isfinite(d_rel):
        continue

      pt = map_to_screen(d_rel, -y_rel, path_offset_z)
      if pt is None:
        continue

      x, y = pt
      color = LEAD_TRACK_COLOR if track.trackId == lead_track_id else RADAR_TRACK_COLOR
      rl.draw_circle(int(x), int(y), track_size, color)
