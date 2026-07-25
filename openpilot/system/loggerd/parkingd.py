#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import random
import shutil
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware import HARDWARE
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.version import get_version
from openpilot.system.loggerd.config import PARKING_BUFFER_DIR
from openpilot.system.loggerd.parking_settings import get_int, put_int

BUFFER_SECONDS = 120.0
POST_MOTION_SECONDS = 60.0
CHUNK_SECONDS = 5.0
SEGMENT_SECONDS = 60.0
V4L2_BUF_FLAG_KEYFRAME = 0x8
CHUNK_MAGIC = b"PKH2"
TS_PACKET_SIZE = 188
TS_PAT_PID = 0x0000
TS_PMT_PID = 0x0100
TS_VIDEO_PID = 0x0101


def _param_text(params: Params, key: str) -> str:
  value = params.get(key)
  if isinstance(value, bytes):
    return value.decode(errors="replace")
  return value or ""


def _last_gps_position(params: Params) -> tuple[float, float, float] | None:
  try:
    position = json.loads(_param_text(params, "LastGPSPositionLLK"))
    return float(position["latitude"]), float(position["longitude"]), float(position.get("altitude", 0.0))
  except (KeyError, TypeError, ValueError, json.JSONDecodeError):
    return None


def _read_ts_frames(path: Path):
  pes = bytearray()
  with path.open("rb") as stream:
    while packet := stream.read(TS_PACKET_SIZE):
      if len(packet) != TS_PACKET_SIZE or packet[0] != 0x47:
        continue
      pid = ((packet[1] & 0x1f) << 8) | packet[2]
      if pid != TS_VIDEO_PID:
        continue
      adaptation_control = (packet[3] >> 4) & 0x03
      if adaptation_control not in (1, 3):
        continue
      offset = 4
      if adaptation_control == 3:
        offset += 1 + packet[offset]
      payload = packet[offset:]
      if packet[1] & 0x40:
        if pes:
          header_size = 9 + pes[8]
          yield bytes(pes[header_size:])
        pes = bytearray(payload)
      else:
        pes.extend(payload)
  if pes:
    header_size = 9 + pes[8]
    yield bytes(pes[header_size:])


def _retime_parking_frames(source: Path) -> list[tuple[int, bytes]]:
  frames: list[tuple[int, bytes]] = []
  start_timestamp_ns = time.monotonic_ns()
  for frame_id, frame in enumerate(_read_ts_frames(source)):
    frames.append((start_timestamp_ns + frame_id * 50_000_000, frame))
  if not frames:
    raise ValueError(f"No video frames in {source}")
  return frames


def _is_keyframe(frame: bytes) -> bool:
  return b"\x00\x00\x00\x01\x65" in frame or b"\x00\x00\x01\x65" in frame


def _split_video_segments(frames: list[tuple[int, bytes]]) -> list[list[tuple[int, bytes]]]:
  segments: list[list[tuple[int, bytes]]] = [[]]
  for frame in frames:
    current = segments[-1]
    if current and (frame[0] - current[0][0]) / 1e9 >= SEGMENT_SECONDS and _is_keyframe(frame[1]):
      segments.append([])
    segments[-1].append(frame)
  return segments


def _write_route_qlog(path: Path, params: Params, start_wall_time_ns: int,
                      frames: list[tuple[int, bytes]], segment_num: int,
                      global_frame_offset: int, final_segment: bool) -> None:
  # Connect discovers routes and segment boundaries from qlogs. Match the
  # loggerd envelope and include qcamera frame indices for the whole segment.
  start_mono_time_ns = frames[0][0]
  end_mono_time_ns = frames[-1][0] + 50_000_000
  init_msg = messaging.new_message("initData", valid=True)
  init_msg.logMonoTime = start_mono_time_ns
  init = init_msg.initData
  init.wallTimeNanos = start_wall_time_ns
  init.version = get_version()
  init.deviceType = HARDWARE.get_device_type()
  init.gitCommit = _param_text(params, "GitCommit")
  init.gitCommitDate = _param_text(params, "GitCommitDate")
  init.gitBranch = _param_text(params, "GitBranch")
  init.gitRemote = _param_text(params, "GitRemote")
  init.dongleId = _param_text(params, "DongleId")
  init.passive = False

  start_msg = messaging.new_message("sentinel", valid=True)
  start_msg.logMonoTime = start_mono_time_ns
  start_msg.sentinel.type = "startOfRoute" if segment_num == 0 else "startOfSegment"

  started_msg = messaging.new_message("deviceState", valid=True)
  started_msg.logMonoTime = start_mono_time_ns
  started_msg.deviceState.deviceType = HARDWARE.get_device_type()
  started_msg.deviceState.started = True
  started_msg.deviceState.startedMonoTime = start_mono_time_ns

  stopped_msg = messaging.new_message("deviceState", valid=True)
  stopped_msg.logMonoTime = end_mono_time_ns
  stopped_msg.deviceState.deviceType = HARDWARE.get_device_type()
  stopped_msg.deviceState.started = False

  end_msg = messaging.new_message("sentinel", valid=True)
  end_msg.logMonoTime = end_mono_time_ns
  end_msg.sentinel.type = "endOfRoute" if final_segment else "endOfSegment"

  gps_position = _last_gps_position(params)
  start_wall_time_ms = start_wall_time_ns // 1_000_000
  tmp = path.with_suffix(".tmp")
  with tmp.open("wb") as output:
    for msg in (init_msg, start_msg, started_msg):
      output.write(msg.to_bytes())
    for segment_frame_id, (timestamp_ns, frame) in enumerate(frames):
      frame_id = global_frame_offset + segment_frame_id
      index_msg = messaging.new_message("qRoadEncodeIdx", valid=True)
      index_msg.logMonoTime = timestamp_ns
      index = index_msg.qRoadEncodeIdx
      index.frameId = frame_id
      index.type = "qcameraH264"
      index.encodeId = frame_id
      index.segmentNum = segment_num
      index.segmentId = segment_frame_id
      index.segmentIdEncode = segment_frame_id
      index.timestampSof = timestamp_ns - 11_000_000
      index.timestampEof = timestamp_ns
      keyframe = _is_keyframe(frame)
      index.flags = 0x80004000 | (V4L2_BUF_FLAG_KEYFRAME if keyframe else 0)
      index.len = len(frame)
      output.write(index_msg.to_bytes())
      if segment_frame_id % 20 == 0:
        camera_msg = messaging.new_message("roadCameraState", valid=True)
        camera_msg.logMonoTime = timestamp_ns + 1_000_000
        camera_msg.roadCameraState.frameId = frame_id
        camera_msg.roadCameraState.timestampSof = timestamp_ns - 11_000_000
        camera_msg.roadCameraState.timestampEof = timestamp_ns
        output.write(camera_msg.to_bytes())

        if gps_position is not None:
          gps_msg = messaging.new_message("gpsLocationExternal", valid=True)
          gps_msg.logMonoTime = timestamp_ns
          gps = gps_msg.gpsLocationExternal
          gps.flags = 1
          gps.latitude, gps.longitude, gps.altitude = gps_position
          gps.horizontalAccuracy = 5.0
          gps.unixTimestampMillis = start_wall_time_ms + frame_id * 50
          gps.source = "qcomdiag"
          gps.vNED = [0.0, 0.0, 0.0]
          gps.hasFix = True
          output.write(gps_msg.to_bytes())
    if final_segment:
      output.write(stopped_msg.to_bytes())
    output.write(end_msg.to_bytes())
  os.replace(tmp, path)


def _mpeg_crc32(data: bytes) -> int:
  crc = 0xffffffff
  for value in data:
    crc ^= value << 24
    for _ in range(8):
      crc = ((crc << 1) ^ 0x04c11db7) & 0xffffffff if crc & 0x80000000 else (crc << 1) & 0xffffffff
  return crc


def _psi_section(data: bytes) -> bytes:
  return data + _mpeg_crc32(data).to_bytes(4, "big")


def _encode_pts(pts: int) -> bytes:
  pts &= (1 << 33) - 1
  return bytes((
    0x21 | (((pts >> 30) & 0x07) << 1),
    (pts >> 22) & 0xff,
    0x01 | (((pts >> 15) & 0x7f) << 1),
    (pts >> 7) & 0xff,
    0x01 | ((pts & 0x7f) << 1),
  ))


def _encode_pcr(pcr_base: int) -> bytes:
  pcr_base &= (1 << 33) - 1
  return bytes((
    (pcr_base >> 25) & 0xff,
    (pcr_base >> 17) & 0xff,
    (pcr_base >> 9) & 0xff,
    (pcr_base >> 1) & 0xff,
    ((pcr_base & 1) << 7) | 0x7e,
    0,
  ))


class MpegTsWriter:
  def __init__(self, output: BinaryIO, fps: int = 20):
    self.output = output
    self.frame_duration = 90000 // fps
    self.frame_count = 0
    self.continuity: dict[int, int] = {}

  def _write_packet(self, pid: int, payload: bytes, payload_start: bool, pcr: int | None = None,
                    random_access: bool = False) -> None:
    continuity = self.continuity.get(pid, 0)
    self.continuity[pid] = (continuity + 1) & 0x0f

    adaptation = b""
    if pcr is not None:
      adaptation_length = 183 - len(payload)
      assert adaptation_length >= 7
      flags = 0x10 | (0x40 if random_access else 0)
      adaptation = bytes((adaptation_length, flags)) + _encode_pcr(pcr) + b"\xff" * (adaptation_length - 7)
      adaptation_control = 3
    elif len(payload) < 184:
      adaptation_length = 183 - len(payload)
      adaptation = bytes((adaptation_length,))
      if adaptation_length:
        adaptation += b"\x00" + b"\xff" * (adaptation_length - 1)
      adaptation_control = 3
    else:
      adaptation_control = 1

    header = bytes((
      0x47,
      ((0x40 if payload_start else 0) | ((pid >> 8) & 0x1f)),
      pid & 0xff,
      (adaptation_control << 4) | continuity,
    ))
    packet = header + adaptation + payload
    assert len(packet) == TS_PACKET_SIZE
    self.output.write(packet)

  def _write_payload(self, pid: int, payload: bytes, pcr: int | None = None, random_access: bool = False) -> None:
    first = True
    while payload:
      capacity = 176 if first and pcr is not None else 184
      chunk, payload = payload[:capacity], payload[capacity:]
      self._write_packet(pid, chunk, first, pcr if first else None, random_access if first else False)
      first = False

  def _write_program_tables(self) -> None:
    pat = _psi_section(bytes.fromhex("00b00d0001c100000001e100"))
    pmt = _psi_section(bytes.fromhex("02b0120001c10000e101f0001be101f000"))
    self._write_payload(TS_PAT_PID, b"\x00" + pat)
    self._write_payload(TS_PMT_PID, b"\x00" + pmt)

  def write_frame(self, data: bytes, pts: int | None = None) -> None:
    if self.frame_count % 20 == 0:
      self._write_program_tables()

    pts = self.frame_count * self.frame_duration if pts is None else pts
    pes = b"\x00\x00\x01\xe0\x00\x00\x80\x80\x05" + _encode_pts(pts) + data
    random_access = b"\x00\x00\x00\x01\x65" in data or b"\x00\x00\x01\x65" in data
    self._write_payload(TS_VIDEO_PID, pes, pcr=pts, random_access=random_access)
    self.frame_count += 1


@dataclass(frozen=True)
class Chunk:
  path: Path
  started_at: float


class MotionDetector:
  """Detect physical device movement from the accelerometer and gyroscope."""

  def __init__(self) -> None:
    self.accel_baseline: list[float] | None = None
    self.warmup_samples = 0
    self.imu_hits = 0

  def update_imu(self, acceleration: list[float] | None, gyro: list[float] | None) -> bool:
    if acceleration is not None:
      if self.accel_baseline is None:
        self.accel_baseline = acceleration.copy()
      delta = math.sqrt(sum((v - b) ** 2 for v, b in zip(acceleration, self.accel_baseline, strict=True)))
      alpha = 0.002 if delta < 0.5 else 0.0001
      self.accel_baseline = [(1.0 - alpha) * b + alpha * v for b, v in zip(self.accel_baseline, acceleration, strict=True)]
    else:
      delta = 0.0

    gyro_norm = math.sqrt(sum(v * v for v in gyro)) if gyro is not None else 0.0
    self.warmup_samples += 1
    if self.warmup_samples < 200:
      return False

    self.imu_hits = self.imu_hits + 1 if delta > 1.5 or gyro_norm > 0.35 else 0
    return self.imu_hits >= 2

class ParkingRecorder:
  def __init__(self, root: Path, params: Params) -> None:
    self.root = root
    self.params = params
    self.buffer_dir = root / PARKING_BUFFER_DIR
    self.events_dir = self.buffer_dir / "events"
    self.chunks: deque[Chunk] = deque()
    self.current: Chunk | None = None
    self.current_file: BinaryIO | None = None
    self.latest_header = b""
    self.event_dir: Path | None = None
    self.event_route: str | None = None
    self.event_deadline = 0.0
    self.event_chunk_count = 0
    self.remux_threads: list[threading.Thread] = []

    shutil.rmtree(self.buffer_dir, ignore_errors=True)
    self.events_dir.mkdir(parents=True)
    self._migrate_legacy_routes()

  def _migrate_legacy_routes(self) -> None:
    for legacy_dir in self.root.glob("800000*--0"):
      source = legacy_dir / "qcamera.ts"
      if not source.is_file():
        continue
      try:
        if os.getxattr(source, "user.parking_migrated").startswith(b"v4:"):
          continue
      except OSError:
        pass

      route = self._new_route_name()
      try:
        frames = _retime_parking_frames(source)
        duration = len(frames) / 20.0
        start_wall_time_ns = source.stat().st_mtime_ns - int(duration * 1e9)
        self._write_segmented_route(route, frames, start_wall_time_ns)
        os.setxattr(source, "user.parking_migrated", b"v4:" + route.encode())
        self._mark_route_for_upload(route)
        cloudlog.event("parking_route_migrated", source=str(legacy_dir), route=route)
      except Exception:
        cloudlog.exception("failed to migrate legacy parking route")

  def _write_segmented_route(self, route: str, frames: list[tuple[int, bytes]],
                             start_wall_time_ns: int) -> list[Path]:
    segments = _split_video_segments(frames)
    output_dirs: list[Path] = []
    locks: list[Path] = []
    global_frame_offset = 0
    try:
      for segment_num, segment_frames in enumerate(segments):
        output_dir = self.root / f"{route}--{segment_num}"
        output_dir.mkdir()
        output_dirs.append(output_dir)
        lock = output_dir / "qcamera.ts.lock"
        lock.touch()
        locks.append(lock)

        with (output_dir / "qcamera.ts").open("wb") as output_file:
          writer = MpegTsWriter(output_file)
          for timestamp_eof_ns, frame in segment_frames:
            writer.write_frame(frame, timestamp_eof_ns * 90000 // 1_000_000_000)

        _write_route_qlog(output_dir / "qlog", self.params, start_wall_time_ns, segment_frames,
                          segment_num, global_frame_offset, segment_num == len(segments) - 1)
        global_frame_offset += len(segment_frames)
      return output_dirs
    except Exception:
      for output_dir in output_dirs:
        shutil.rmtree(output_dir, ignore_errors=True)
      raise
    finally:
      for lock in locks:
        lock.unlink(missing_ok=True)

  def _new_route_name(self) -> str:
    count = get_int(self.params, "RouteCount")
    put_int(self.params, "RouteCount", count + 1)
    return f"{count & 0xffffffff:08x}--{random.randbytes(5).hex()}"

  def _mark_route_for_upload(self, route: str) -> None:
    routes = [r for r in (self.params.get("AthenadRecentlyViewedRoutes") or "").split(",") if r]
    if route not in routes:
      self.params.put("AthenadRecentlyViewedRoutes", ",".join([*routes[-99:], route]))

  def _start_chunk(self, now: float, header: bytes) -> None:
    path = self.buffer_dir / f"chunk-{time.monotonic_ns()}.h264"
    self.current_file = path.open("wb")
    self.current_file.write(CHUNK_MAGIC)
    self.current_file.write(struct.pack(">I", len(header)))
    self.current_file.write(header)
    self.current = Chunk(path, now)
    self.chunks.append(self.current)
    if self.event_dir is not None:
      self._link_event_chunk(path)

  def _close_chunk(self) -> None:
    if self.current_file is not None:
      self.current_file.flush()
      self.current_file.close()
    self.current_file = None
    self.current = None

  def _link_event_chunk(self, path: Path) -> None:
    assert self.event_dir is not None
    destination = self.event_dir / f"{self.event_chunk_count:04d}.h264"
    if not destination.exists():
      os.link(path, destination)
      self.event_chunk_count += 1

  def _trim_buffer(self, now: float) -> None:
    while len(self.chunks) > 1 and self.chunks[1].started_at < now - BUFFER_SECONDS:
      old = self.chunks.popleft()
      # Active events hold hard links to their chunks, so unlinking the ring
      # entry cannot remove footage that is waiting to be remuxed.
      old.path.unlink(missing_ok=True)

  def trigger(self, now: float, reason: str) -> None:
    self.event_deadline = max(self.event_deadline, now + POST_MOTION_SECONDS)
    if self.event_dir is not None:
      return

    self.event_route = self._new_route_name()
    self.event_dir = self.events_dir / self.event_route
    self.event_dir.mkdir()
    self.event_chunk_count = 0
    for chunk in self.chunks:
      if chunk.started_at >= now - BUFFER_SECONDS - CHUNK_SECONDS:
        self._link_event_chunk(chunk.path)
    cloudlog.event("parking_motion_detected", reason=reason, route=self.event_route)

  def add_packet(self, data: bytes, header: bytes, keyframe: bool, timestamp_eof_ns: int, now: float) -> None:
    if header:
      self.latest_header = header
    if self.current is None:
      if not keyframe or not self.latest_header:
        return
      self._start_chunk(now, self.latest_header)
    elif keyframe and now - self.current.started_at >= CHUNK_SECONDS:
      self._close_chunk()
      self._start_chunk(now, self.latest_header)

    assert self.current_file is not None
    self.current_file.write(struct.pack(">QI", timestamp_eof_ns, len(data)))
    self.current_file.write(data)
    self._trim_buffer(now)

    if self.event_dir is not None and now >= self.event_deadline and keyframe:
      self._close_chunk()
      self._finish_event()

  def _finish_event(self) -> None:
    assert self.event_dir is not None and self.event_route is not None
    event_dir, route = self.event_dir, self.event_route
    self.event_dir = None
    self.event_route = None
    self.event_deadline = 0.0
    self.event_chunk_count = 0

    thread = threading.Thread(target=self._remux_event, args=(event_dir, route), daemon=True)
    thread.start()
    self.remux_threads.append(thread)

  def _remux_event(self, event_dir: Path, route: str) -> None:
    try:
      frames: list[tuple[int, bytes]] = []
      for chunk in sorted(event_dir.glob("*.h264")):
        with chunk.open("rb") as chunk_file:
          if chunk_file.read(len(CHUNK_MAGIC)) != CHUNK_MAGIC:
            raise ValueError(f"Invalid parking chunk {chunk}")
          header_size_data = chunk_file.read(4)
          if len(header_size_data) != 4:
            raise ValueError(f"Truncated parking chunk header {chunk}")
          header = chunk_file.read(struct.unpack(">I", header_size_data)[0])
          first_frame = True
          while frame_metadata := chunk_file.read(12):
            if len(frame_metadata) != 12:
              raise ValueError(f"Truncated parking frame metadata {chunk}")
            timestamp_eof_ns, frame_size = struct.unpack(">QI", frame_metadata)
            frame = chunk_file.read(frame_size)
            if len(frame) != frame_size:
              raise ValueError(f"Truncated parking frame {chunk}")
            frames.append((timestamp_eof_ns, (header if first_frame else b"") + frame))
            first_frame = False
      if not frames:
        raise ValueError("Parking event contained no video frames")

      duration = len(frames) / 20.0
      start_wall_time_ns = time.time_ns() - int(duration * 1e9)
      output_dirs = self._write_segmented_route(route, frames, start_wall_time_ns)
      self._mark_route_for_upload(route)
      cloudlog.event("parking_recording_saved", route=route, segments=len(output_dirs))
    except Exception:
      cloudlog.exception("failed to save parking recording")
    finally:
      shutil.rmtree(event_dir, ignore_errors=True)

  def close(self) -> None:
    self._close_chunk()
    if self.event_dir is not None:
      self._finish_event()
    for thread in self.remux_threads:
      thread.join(timeout=35)


def main() -> None:
  params = Params()
  recorder = ParkingRecorder(Path(Paths.log_root()), params)
  detector = MotionDetector()
  video_service = "livestreamRoadEncodeData"
  sm = messaging.SubMaster([video_service, "accelerometer", "gyroscope"], poll=video_service)

  try:
    while True:
      sm.update(1000)
      now = time.monotonic()

      acceleration = list(sm["accelerometer"].acceleration.v) if sm.updated["accelerometer"] else None
      gyro = list(sm["gyroscope"].gyroUncalibrated.v) if sm.updated["gyroscope"] else None
      if detector.update_imu(acceleration, gyro):
        recorder.trigger(now, "imu")

      if not sm.updated[video_service]:
        continue
      packet = sm[video_service]
      keyframe = bool(packet.idx.flags & V4L2_BUF_FLAG_KEYFRAME)
      data = bytes(packet.data)
      recorder.add_packet(data, bytes(packet.header), keyframe, int(packet.idx.timestampEof), now)
  finally:
    recorder.close()


if __name__ == "__main__":
  main()
