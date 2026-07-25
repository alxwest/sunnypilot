import io
from pathlib import Path

import openpilot.system.loggerd.parkingd as parkingd
from openpilot.system.loggerd.parkingd import MotionDetector, ParkingRecorder


def test_imu_motion_detector_ignores_stationary_gravity():
  detector = MotionDetector()
  assert not any(detector.update_imu([0.0, 0.0, 9.81], [0.0, 0.0, 0.0]) for _ in range(300))


def test_imu_motion_detector_triggers_after_warmup():
  detector = MotionDetector()
  for _ in range(250):
    detector.update_imu([0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
  assert not detector.update_imu([2.0, 0.0, 9.81], [0.0, 0.0, 0.0])
  assert detector.update_imu([2.0, 0.0, 9.81], [0.0, 0.0, 0.0])


def test_video_motion_detector_requires_sustained_complexity_change():
  detector = MotionDetector()
  for _ in range(100):
    assert not detector.update_video(1000, False)
  for _ in range(5):
    assert not detector.update_video(3000, False)
  assert detector.update_video(3000, False)


def test_recorder_keeps_prebuffer_and_saves_post_motion(tmp_path, monkeypatch):
  class FakeParams:
    def __init__(self):
      self.values = {}

    def check_key(self, key):
      return key

    def get(self, key):
      return self.values.get(key)

    def put(self, key, value, block=False):
      self.values[key] = value

  class FakeProcess:
    def __init__(self, cmd, stdin):
      self.stdin = io.BytesIO()
      Path(cmd[-1]).touch()
      self.returncode = None

    def wait(self, timeout=None):
      self.returncode = 0
      return self.returncode

    def poll(self):
      return self.returncode

    def kill(self):
      self.returncode = -9

  monkeypatch.setattr(parkingd.subprocess, "Popen", FakeProcess)
  params = FakeParams()
  recorder = ParkingRecorder(tmp_path, params)

  for second in range(131):
    keyframe = second % 5 == 0
    recorder.add_packet(b"frame", b"header" if keyframe else b"", keyframe, float(second))

  recorder.trigger(130.0, "test")
  assert recorder.event_dir is not None
  assert len(list(recorder.event_dir.glob("*.h264"))) >= 24

  for second in range(131, 192):
    keyframe = second % 5 == 0
    recorder.add_packet(b"frame", b"header" if keyframe else b"", keyframe, float(second))
  recorder.close()

  assert len(list(tmp_path.glob("*--0/qcamera.ts"))) == 1
  assert params.values["AthenadRecentlyViewedRoutes"]
