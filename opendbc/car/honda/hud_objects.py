from dataclasses import dataclass

from opendbc.can.parser import CANParser

NUM_SLOTS = 10

# ---- Receive: camera HUD_OBJECTS -> snapshot --------------------------------
# Empty slot sentinel is max value of 1023, or 196.9 m after scaling
LONG_DIST_CAP_M = 195.0


@dataclass
class HudObject:
  slot: int
  object_id: int     # 1..31 when valid, 0 when empty
  d_rel: float       # longitudinal distance from ego (m)
  y_rel: float       # lateral position (+left) of ego (m)
  is_lead_car: bool  # camera's lead tag (slot 0 when present)
  valid: bool
  car_type: int = -1      # 7 = CAR, -7 = TRUCK, -1 = inactive
  rotation: int = -128    # 0 straight, <0 left, >0 right, -128 = inactive


class HudObjectTracker:
  """Persists the 10 HUD_OBJECTS slots"""

  def __init__(self):
    self._tracks: list[HudObject] = [
      HudObject(slot=i, object_id=0, d_rel=0.0, y_rel=0.0, is_lead_car=False, valid=False)
      for i in range(NUM_SLOTS)
    ]

  def update(self, cp_cam: CANParser) -> None:
    # HUD_OBJECTS is one message multiplexed over 10 slots x 4 banks (MUX = (bank<<4)|slot, slot 1..10),
    # each frame carrying one slot at fixed bit positions. Using vl_all to catch any missed frames, though not strictly
    # necessary since update() is called at 100Hz and this signal updates at 50Hz.
    # (Touching vl first registers the message.)
    _ = cp_cam.vl["HUD_OBJECTS"]
    vla = cp_cam.vl_all["HUD_OBJECTS"]

    muxes = vla["MUX"]
    obj_ids = vla["OBJECT_ID"]
    long_dists = vla["LONG_DIST"]
    lat_dists = vla["LAT_DIST"]
    lead_flags = vla["IS_LEAD_CAR"]
    car_types = vla["CAR_TYPE"]
    rotations = vla["ROTATION"]

    for mux, oid, ld, yd, lead, ct, rot in zip(muxes, obj_ids, long_dists, lat_dists, lead_flags,
                                               car_types, rotations, strict=True):
      slot = (int(mux) - 1) % 16
      if 0 <= slot < NUM_SLOTS:
        valid = oid != 0 and ld < LONG_DIST_CAP_M
        self._tracks[slot] = HudObject(
          slot=slot,
          object_id=int(oid),
          d_rel=float(ld),
          y_rel=float(yd),
          is_lead_car=bool(lead),
          valid=valid,
          car_type=int(ct),
          rotation=int(rot),
        )

  def snapshot(self) -> list[HudObject]:
    return self._tracks
