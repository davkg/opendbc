"""
Aggregates the per-slot tracks from the Honda Bosch radarless camera's
CAMERA_OBJECT_TRACKS message (id 0x6CD1C97 / 114120023) into a snapshot
the rest of the stack can consume.

Message structure (reverse-engineered):
  TRACK_INDEX = (bank << 4) | slot, where slot ∈ 1..10 and bank ∈ 0..3.
  Each of the 10 object slots is transmitted 4× per cycle (~5 Hz/object),
  with bank 0 (mux values 1..10) carrying the most recent values per slot.

For Phase 1+2 we read bank 0 only (~1.3 Hz per slot, sufficient for the
onroad visualization with downstream smoothing). Bank expansion to all
four banks is deferred until the planner needs the higher rate.
"""
from dataclasses import dataclass

from opendbc.can.parser import CANParser


# Raw LONG_DIST cap (1023) × scale 0.1 = 102.3 m. Empty slots park here.
# Pick a threshold just under it so a real ~100 m track still reads as valid.
LONG_DIST_CAP_M = 102.0

NUM_SLOTS = 10


@dataclass
class CameraObjectTrack:
  slot: int        # 0..9
  object_id: int   # raw OBJECT_ID, 1..31 when valid, 0 when empty
  d_rel: float     # longitudinal distance from ego (m)
  y_rel: float     # lateral position, +left of ego (m)
  valid: bool      # True if slot currently carries a real track


class CameraObjectTracker:
  """Stateless wrapper over CAMERA_OBJECT_TRACKS bank-0 signals.

  Held as an attribute on the Honda CarState instance and updated once per
  carstate tick. `snapshot()` returns the current 10-slot table; consumers
  filter by `.valid`.
  """

  def __init__(self):
    self._tracks: list[CameraObjectTrack] = [
      CameraObjectTrack(slot=i, object_id=0, d_rel=0.0, y_rel=0.0, valid=False)
      for i in range(NUM_SLOTS)
    ]

  def update(self, cp_cam: CANParser) -> None:
    msg = cp_cam.vl["CAMERA_OBJECT_TRACKS"]
    for i in range(NUM_SLOTS):
      n = i + 1  # signals are T1..T10
      obj_id = int(msg[f"T{n}_OBJECT_ID"])
      d_rel = float(msg[f"T{n}_LONG_DIST"])
      y_rel = float(msg[f"T{n}_LAT_DIST"])
      valid = obj_id != 0 and d_rel < LONG_DIST_CAP_M
      self._tracks[i] = CameraObjectTrack(
        slot=i,
        object_id=obj_id,
        d_rel=d_rel,
        y_rel=y_rel,
        valid=valid,
      )

  def snapshot(self) -> list[CameraObjectTrack]:
    return self._tracks
