# Render OpenPilot's detected lead car on the Honda Bosch radarless dash via the CAMERA_OBJECT_TRACKS message.
#
# CAMERA_OBJECT_TRACKS (id 0x6CD5557 / 114120023) is the camera's adjacent-vehicle table: TRACK_INDEX
# multiplexes 10 object slots across 4 banks (1-10, 17-26, 33-42, 49-58 -> ~5 Hz/slot), each frame carrying
# one slot at fixed bit positions. The lead, when present, is always track index 1 (slot 0) with IS_LEAD_CAR=1.
# Structurally identical to LANE_PATH (see lane_path.py): MUX + m1 signals + honda CHECKSUM/COUNTER, so the
# encoder mirrors create_lane_path -- the CANPacker computes the checksum/counter for the extended ID.
#
# Phase 1: replace ONLY slot 0 with OP's lead (radarState.leadOne); slots 1-9 are sent inactive (the camera's
# empty-slot sentinel). check_relay statically blocks the camera's whole copy, so to keep showing the stock
# non-lead cars we'd have to forward them too -- that's Phase 2.
import numpy as np

NUM_SLOTS = 10

# Full TRACK_INDEX cycle the camera emits: 10 slots across 4 redundant banks (bank-major so each slot refreshes
# evenly across the cycle), mirroring lane_path.MUX_CYCLE. Slot = (track_index - 1) % 16.
TRACK_INDEX_CYCLE = tuple(slot + bank * 16 for bank in range(4) for slot in range(1, NUM_SLOTS + 1))

# Physical values the camera sends for an EMPTY slot (decoded from stock CAMERA_OBJECT_TRACKS). Sending these
# byte-faithfully marks a slot empty; an inconsistent frame risks the dash rejecting it (cf the LKAS_HUD_2 freeze).
INACTIVE = {
  "OBJECT_ID": 0,
  "IS_LEAD_CAR": 0,
  "CAR_TYPE": -1,        # VAL_ -1 "INACTIVE"
  "ROTATION": -128,      # "−128 when inactive"
  "LONG_DIST": 196.9,    # raw 1023 under (0.209, −16.9) = empty
  "LAT_DIST": 204.7,     # max, "when inactive"
}

CAR_TYPE_CAR = 7         # VAL_ 7 "CAR"
LONG_DIST_MAX_M = 194.0  # keep an active lead below the 196.9 empty sentinel so it stays "valid"
LAT_DIST_LIM_M = 204.7   # 12-bit signed @0.1 -> ±204.x m

# Lead-identity re-ID (LeadTrackId): OP exposes no persistent lead identity, so we mint a new OBJECT_ID when the
# lead first appears and when its range moves inconsistently with vRel for a sustained moment -- a handoff to a
# different car (observed on routes 000000ea seg42 / 000000e9 seg17 as a ~8-11 m/s range-rate anomaly over ~2 s
# while vRel stayed ~0/±1). dRel is noisy (the vision lead jitters several metres), so a per-sample range-rate
# test churns; instead we run a leaky position predictor -- feed-forward vRel, slowly leak toward the measured
# dRel -- and re-id only when the residual ACCUMULATES past a gap (a sustained anomaly, not jitter). Stable id
# while the lead tracks continuously; the dash re-places the icon on id change.
REID_GAP_M = 8.0         # m, accumulated |dRel − predicted| above this = a different car
REID_TAU = 1.5           # s, leak time-constant (predictor follows real/slow changes, lags a sudden handoff)
REID_REFRACTORY = 1.5    # s, collapse the multi-frame transition into one re-id
MAX_OBJECT_ID = 31       # OBJECT_ID is 5-bit (1..31; 0 = empty)


class LeadTrackId:
  """Tracks a stable slot-0 OBJECT_ID for OP's lead, re-IDing on a fresh lead or a range discontinuity."""
  def __init__(self):
    self.object_id = 0
    self._on = False
    self._pred = 0.0      # leaky predicted dRel
    self._prev_t = 0.0
    self._reid_t = -1e9

  def update(self, status: bool, d_rel: float, v_rel: float, now: float) -> int:
    """Returns the OBJECT_ID to send for slot 0 (0 when there's no lead)."""
    if not status:
      self.object_id = 0
      self._on = False
      return 0

    new_lead = not self._on
    if self._on:
      dt = max(now - self._prev_t, 1e-3)
      self._pred += v_rel * dt                          # feed-forward: d(dRel)/dt ~= vRel
      self._pred += min(dt / REID_TAU, 1.0) * (d_rel - self._pred)   # leak toward the measurement
      if abs(d_rel - self._pred) > REID_GAP_M and now - self._reid_t > REID_REFRACTORY:
        new_lead = True
    self._prev_t = now

    if new_lead:
      self.object_id = self.object_id % MAX_OBJECT_ID + 1   # wrap 1..31, never 0
      self._reid_t = now
      self._pred = d_rel                                    # reset the predictor to the new lead
    self._on = True
    return self.object_id


def create_object_track(packer, bus, track_index, track):
  """Pack one CAMERA_OBJECT_TRACKS frame for TRACK_INDEX `track_index`.

  `track` is None for an inactive slot, else a dict {d_rel, y_rel, object_id}: OP's lead, mapped to
  LONG_DIST=dRel, LAT_DIST=yRel (both leadOne.yRel and the dash's LAT_DIST are +left -- confirmed: a right-lane
  lead reads stock LAT_DIST ~−3.2), IS_LEAD_CAR=1, ROTATION=0 (OP has no lead heading -- Phase 2), CAR_TYPE=CAR.
  CHECKSUM (honda) + COUNTER are computed by the packer.

  NOTE: leadOne.yRel is heavily path-centered (only ~−0.2 m while a lead sits 2.3 m to the side), so the icon
  renders near-centre on the correct side; true off-path lateral magnitude is a Phase-2 item (the model under-
  reports it -- leadsV3[0].y is identical in magnitude, just sign-flipped, so no better).
  """
  values = {"TRACK_INDEX": track_index}
  if track is None:
    values.update(INACTIVE)
  else:
    values.update({
      "OBJECT_ID": int(track["object_id"]),
      "IS_LEAD_CAR": 1,
      "CAR_TYPE": CAR_TYPE_CAR,
      "ROTATION": 0,
      "LONG_DIST": float(np.clip(track["d_rel"], 0.0, LONG_DIST_MAX_M)),
      "LAT_DIST": float(np.clip(track["y_rel"], -LAT_DIST_LIM_M, LAT_DIST_LIM_M)),
    })
  return packer.make_can_msg("CAMERA_OBJECT_TRACKS", bus, values)
