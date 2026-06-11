"""
Aggregates the per-slot tracks from the Honda Bosch radarless camera's
CAMERA_OBJECT_TRACKS message (id 0x6CD5557 / 114120023) into a snapshot
the rest of the stack can consume.

Message structure (reverse-engineered):
  TRACK_INDEX = (bank << 4) | slot, where slot ∈ 1..10 and bank ∈ 0..3.
  Each of the 10 object slots is transmitted 4× per cycle (~5 Hz/object),
  spread across all four banks.

opendbc's CANParser doesn't gate multiplexed signals by their mux value —
T1..T10_OBJECT_ID (and the LONG_DIST / LAT_DIST families) all alias the
same physical bits, so reading them via `cp.vl[...]["Tn_*"]` returns the
same value 10 times. We instead iterate `cp.vl_all[...]` (the list of all
frames since the last update), read `TRACK_INDEX` per frame, and dispatch
each frame's data to the correct slot ourselves. This naturally covers
all four banks for ~5 Hz/slot.
"""
from dataclasses import dataclass

from opendbc.can.parser import CANParser


# Empty slots park raw LONG_DIST at 1023, which under our scaling (0.209, -16.9)
# is 196.9 m. Pick a threshold safely above any plausible real track but
# below the empty-slot value so real tracks remain valid.
LONG_DIST_CAP_M = 195.0

NUM_SLOTS = 10


@dataclass
class CameraObjectTrack:
  slot: int        # 0..9
  object_id: int   # raw OBJECT_ID, 1..31 when valid, 0 when empty
  d_rel: float     # longitudinal distance from ego (m)
  y_rel: float     # lateral position, +left of ego (m)
  valid: bool      # True if slot currently carries a real track


class CameraObjectTracker:
  """Aggregator over CAMERA_OBJECT_TRACKS. Slot state persists across ticks;
  each incoming frame updates exactly one slot (determined by TRACK_INDEX).
  `snapshot()` returns the current 10-slot table; consumers filter by `.valid`.
  """

  def __init__(self):
    self._tracks: list[CameraObjectTrack] = [
      CameraObjectTrack(slot=i, object_id=0, d_rel=0.0, y_rel=0.0, valid=False)
      for i in range(NUM_SLOTS)
    ]

  def update(self, cp_cam: CANParser) -> None:
    # Touch vl to ensure the message is registered with the parser; subsequent
    # cp.update() calls will then populate vl_all for this address.
    _ = cp_cam.vl["CAMERA_OBJECT_TRACKS"]
    vla = cp_cam.vl_all["CAMERA_OBJECT_TRACKS"]

    # T1..T10_* all alias the same bit positions (multiplex value is dropped by
    # the DBC loader). Read any single Tn_* family.
    indices = vla["TRACK_INDEX"]
    obj_ids = vla["T1_OBJECT_ID"]
    long_dists = vla["T1_LONG_DIST"]
    lat_dists = vla["T1_LAT_DIST"]

    for ti, oid, ld, yd in zip(indices, obj_ids, long_dists, lat_dists, strict=True):
      slot = (int(ti) - 1) % 16  # TRACK_INDEX = (bank<<4)|slot, slot ∈ 1..10
      if 0 <= slot < NUM_SLOTS:
        valid = oid != 0 and ld < LONG_DIST_CAP_M
        self._tracks[slot] = CameraObjectTrack(
          slot=slot,
          object_id=int(oid),
          d_rel=float(ld),
          y_rel=float(yd),
          valid=valid,
        )

  def snapshot(self) -> list[CameraObjectTrack]:
    return self._tracks
