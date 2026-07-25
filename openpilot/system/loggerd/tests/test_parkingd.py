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


def test_recorder_keeps_prebuffer_and_saves_post_motion(tmp_path):
  class FakeParams:
    def __init__(self):
      self.values = {}

    def check_key(self, key):
      return key

    def get(self, key):
      return self.values.get(key)

    def put(self, key, value, block=False):
      self.values[key] = value

  params = FakeParams()
  recorder = ParkingRecorder(tmp_path, params)
  header = b"\x00\x00\x00\x01\x67\x64\x00\x1f\x00\x00\x00\x01\x68\xee\x3c\x80"
  frame = b"\x00\x00\x00\x01\x65\x88\x84"

  for second in range(131):
    keyframe = second % 5 == 0
    recorder.add_packet(frame, header if keyframe else b"", keyframe, second * 1_000_000_000, float(second))

  recorder.trigger(130.0, "test")
  assert recorder.event_dir is not None
  assert len(list(recorder.event_dir.glob("*.h264"))) >= 24

  for second in range(131, 192):
    keyframe = second % 5 == 0
    recorder.add_packet(frame, header if keyframe else b"", keyframe, second * 1_000_000_000, float(second))
  recorder.close()

  outputs = list(tmp_path.glob("*--0/qcamera.ts"))
  assert len(outputs) == 1
  route = outputs[0].parent.name.rsplit("--", 1)[0]
  segments = sorted(tmp_path.glob(f"{route}--*/qcamera.ts"))
  assert len(segments) == 4
  for segment in segments:
    assert (segment.parent / "qlog").is_file()
    data = segment.read_bytes()
    assert len(data) % 188 == 0
    assert all(data[i] == 0x47 for i in range(0, len(data), 188))
  assert params.values["AthenadRecentlyViewedRoutes"]
