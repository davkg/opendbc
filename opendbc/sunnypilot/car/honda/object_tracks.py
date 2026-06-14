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
import math

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

# The dash renders LAT_DIST in the EGO frame (the road curve IS in the lateral) but UNDER-SCALES it ~0.3x vs
# reality -- measured stock-only (stock LAT_DIST vs the camera's own LANE_PATH curve and steering k*d^2/2,
# |corr| 0.79, slope ~0.3; NOT road-relative). Our lane (LANE_PATH) is truthful (1:1), so we scale the lead's
# true ego lateral (yRel) toward the stock factor to land the marker ON our lane; raw 1.0x overshoots ~3x (lead
# drawn outside the lane on a steep curve). On-device knob: 0.3 = stock-matched (slightly under-reaches on
# curves), up toward 1.0 reaches further.
LAT_SCALE = 0.35         # multiplies leadOne.yRel -> dash LAT_DIST

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


# Lead-marker SMOOTHING (LeadSmoother): the vision lead's dRel/yRel jitter on the dash -- ~0.1 m of dRel shimmer
# when stopped behind a lead, and large OUTLIER spikes (~30% of distant frames jump >1 m, up to ~2.6 m) on a far
# lead -- while lateral yRel jitter is ~0.1 m far / negligible near (route 00000005 seg1, lead_marker_design.py).
# We smooth what we render WITHOUT lagging real motion:
#   dRel: feed vRel forward (d(dRel)/dt = vRel) so an approaching / pulling-away lead has NO lag, then leak
#         toward the measurement to reject jitter. The leak's input residual is clamped so a single spike can't
#         yank the icon, and the feed-forward is GATED off near zero vRel so a STOPPED lead is a pure low-pass
#         (most stable -- doesn't inject the small standing vRel noise, ~0.28 m/s).
#   yRel: plain low-pass (lateral has no clean velocity to feed forward and moves slowly).
# Reset (snap, don't slide) when the lead changes identity so the icon jumps to the new car. Deliberately SEPARATE
# from LeadTrackId: re-ID wants to DETECT discontinuities, the display wants to HIDE them -- opposite goals.
DREL_SMOOTH_TAU = 0.6     # s, dRel leak time-constant (jitter/drift rejection; vRel FF carries the real motion)
YREL_SMOOTH_TAU = 0.5     # s, yRel low-pass time-constant
FF_VREL_MIN = 0.5         # m/s, feed vRel forward only above this -> stopped lead is a pure low-pass
DREL_RESID_CLAMP = 1.5    # m, cap the measurement residual fed to the leak so an outlier spike barely moves it


class LeadSmoother:
  """Smooths the lead's (dRel, yRel) for a stable dash marker. Call update() per TX tick; it snaps on a new id."""
  def __init__(self):
    self._id = 0
    self._d = 0.0
    self._y = 0.0
    self._t = 0.0

  def update(self, d_rel: float, y_rel: float, v_rel: float, object_id: int, now: float) -> tuple[float, float]:
    """Returns the smoothed (d_rel, y_rel) to render. `object_id` is LeadTrackId's id -- a change = a new car."""
    if object_id != self._id:                 # fresh lead / handoff -> snap to it, don't slide across
      self._id, self._d, self._y, self._t = object_id, d_rel, y_rel, now
      return d_rel, y_rel
    dt = max(now - self._t, 1e-3)
    self._t = now
    if abs(v_rel) >= FF_VREL_MIN:              # feed-forward real motion (no lag); off near zero -> pure low-pass
      self._d += v_rel * dt
    resid = min(max(d_rel - self._d, -DREL_RESID_CLAMP), DREL_RESID_CLAMP)   # clamp -> spike reject
    self._d += (1.0 - math.exp(-dt / DREL_SMOOTH_TAU)) * resid
    self._y += (1.0 - math.exp(-dt / YREL_SMOOTH_TAU)) * (y_rel - self._y)
    return self._d, self._y


def create_object_track(packer, bus, track_index, track):
  """Pack one CAMERA_OBJECT_TRACKS frame for TRACK_INDEX `track_index`.

  `track` is None for an inactive slot, else a dict {d_rel, y_rel, object_id}: OP's lead, mapped to LONG_DIST=dRel
  and LAT_DIST=y_rel (the scaled ego-frame lateral from LeadObjectTrack), +left like the dash's LAT_DIST (a
  right-lane lead reads stock LAT_DIST ~−3.2). IS_LEAD_CAR=1, ROTATION=0 (OP has no lead heading -- Phase 2),
  CAR_TYPE=CAR. CHECKSUM (honda) + COUNTER are computed by the packer.

  NOTE: the dash is EGO-FRAME (the road curve IS in the lateral) but UNDER-SCALES it ~0.3x -- confirmed
  stock-only (cf seg22-25/29: stock LAT_DIST vs both the camera's own LANE_PATH curve and steering k*d^2/2,
  |corr| 0.79, slope ~0.3). It is NOT road-relative. So LeadObjectTrack sends LAT_SCALE*yRel (the lead's true
  ego lateral, scaled to land on our truthful lane). leadsV3[0].y = -yRel exactly (corr −1.00, opposite frame).
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
      "LONG_DIST": min(max(track["d_rel"], 0.0), LONG_DIST_MAX_M),
      "LAT_DIST": min(max(track["y_rel"], -LAT_DIST_LIM_M), LAT_DIST_LIM_M),
    })
  return packer.make_can_msg("CAMERA_OBJECT_TRACKS", bus, values)


class LeadObjectTrack:
  """OP's lead on the dash, end to end: owns the stable slot-0 OBJECT_ID (LeadTrackId), the dRel/yRel smoothing
  (LeadSmoother), and the TRACK_INDEX cycle. The carcontroller just calls update() once per ~50 Hz LANE_PATH
  tick and sends the returned frame."""
  def __init__(self):
    self._track_id = LeadTrackId()
    self._smoother = LeadSmoother()

  def update(self, packer, bus, lead, frame: int, now: float):
    """`lead` = carControlSP.leadOne (status/dRel/yRel/vRel); `frame` = the carcontroller frame counter. Returns
    one packed CAMERA_OBJECT_TRACKS can_msg: OP's lead in slot 0 when the cycle lands on a slot-0 index, else an
    inactive slot (slots 1-9 stay empty in Phase 1). Runs the re-ID + smoothing every tick so their dt-aware
    state stays continuous, even on the frames that pack a non-lead slot."""
    obj_id = self._track_id.update(lead.status, lead.dRel, lead.vRel, now)
    # The dash is EGO-FRAME (the road curve is already in yRel, so the marker rides the lane through curves) but
    # under-scales the lateral ~0.3x vs reality. Scale yRel by LAT_SCALE so the marker lands ON our truthful lane
    # instead of overshooting outside it (raw 1.0x rendered ~3x too far on a steep curve). LAT_SCALE is the knob.
    d_rel, y_rel = self._smoother.update(lead.dRel, LAT_SCALE * lead.yRel, lead.vRel, obj_id, now)
    track_index = TRACK_INDEX_CYCLE[(frame // 2) % len(TRACK_INDEX_CYCLE)]
    on_lead_slot = (track_index - 1) % 16 == 0          # slot 0 = the lead
    track = {"d_rel": d_rel, "y_rel": y_rel, "object_id": obj_id} if (on_lead_slot and lead.status) else None
    return create_object_track(packer, bus, track_index, track)
