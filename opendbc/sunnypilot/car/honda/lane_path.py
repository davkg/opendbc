# Render OpenPilot's lane onto the Honda Bosch radarless dash via the LANE_PATH + LKAS_HUD_2 messages.
#
# LANE_PATH is the camera's lane geometry as 40 lateral offsets (10 MUX indices x 4 sub-offsets,
# near->far), sent ~50 Hz cycling MUX over 4 redundant banks. LKAS_HUD_2 (5 Hz) is the dash's
# lane-display enable/length. We DON'T reproduce the camera's (smoothed, slow) path -- we feed OP's
# own, more responsive lane through the same wire format.
#
# Geometry: the 40 points are lateral offsets at fixed look-ahead distances, so on a curve the lateral
# grows ~0.5*k*d^2 -- evenly spaced distances give tiny near offsets and large far ones for free. The
# per-point gain maps the lane center to raw: raw_i = -GAIN[i] * lateral(LOOKAHEAD[i]).
# Knobs (dialed in on-device):
#   D_MAX  -- how far ahead the points reach
#   GAIN   -- per-point raw units per meter of lane-center lateral (matched to the stock encoding)
# Sign is settled: stock offset = -OP lateral.
import numpy as np

NUM_INDICES = 10
OFFSETS_PER_INDEX = 4
NUM_PTS = NUM_INDICES * OFFSETS_PER_INDEX            # 40, near->far in (mux-1)*4 + offset order

# Full MUX cycle the camera emits: 10 logical indices across 4 redundant banks
# (1-10, 17-26, 33-42, 49-58). The banks carry identical data and together provide the per-index
# update rate, so we emit all four (bank-major -> each index refreshes evenly across the cycle).
MUX_CYCLE = tuple(idx + bank * 16 for bank in range(4) for idx in range(1, NUM_INDICES + 1))

# Clamp valid offsets below the 2047 sentinel so the dash renders them (real geometry stays within
# ~+-340; this is just a guard against out-of-range values).
OFFSET_UNAVAILABLE = 2047                            # camera's "no point here" sentinel (12-bit signed max)
OFFSET_VALID_MAX = 2046

# --- the tunable knobs (calibrate on-device) ---
D_NEAR = 2.0                                         # nearest point look-ahead (m)
D_MAX = 100.0                                        # farthest point look-ahead (m); matches the model path / object-track reach
# The stock LANE_PATH encodes the lane center TRUTHFULLY (confirmed on-road: hugging the left line shows
# the car hugging the line), so we match its encoding. Measured against stock on routes 000000cf seg24/25/29
# (validate_lane_path_scaling.py): centering (DC) gain ~6.8 raw/m -- identified distance-free from the seg29
# lane-change shift (stock pt0 -15 raw / OP -2.2 m) -- and the effective gain RISES with distance to ~11 raw/m
# at 100 m (stock's far points sample/extrapolate ~120-160 m; folding that into a per-point gain at OUR
# distances reproduces stock raw with |corr|>0.9, held-out seg29 amplitude ratio 0.74 vs 0.39 at uniform 4.5).
# Quadratic fit on seg24+25, near anchored at the DC gain. (The SEPARATE object-marker signal,
# CAMERA_OBJECT_TRACKS LAT_DIST, IS under-scaled ~0.35x vs reality, but that's a different message and a
# plan-(b) fix, not the lane.) The dash draws the lane lines at a FIXED width we can't change, so this gain
# sets only the center path's shift + curve. Drive-tunable.
LOOKAHEAD = np.linspace(D_NEAR, D_MAX, NUM_PTS)      # the 40 look-ahead distances, near->far
GAIN = 6.27 + 0.0106 * LOOKAHEAD + 0.000354 * LOOKAHEAD ** 2  # per-point raw/m (~6.3 near -> ~10.9 at 100 m)

# LKAS_HUD_2 lane-display fields (camera values, decoded from logs). We always enable both lines.
LANE_LINE_ON = 3                                     # LEFT_LANE / RIGHT_LANE "shown" value (0 = off)
LANE_LENGTH_MAX_VALUE = 33                           # observed camera max; LANE_LENGTH scales the dash reach -> max = full
LKAS_BOH_1_NEUTRAL = 32                              # coarse road-curvature indicator, 32 = straight (deviates ~+-6 on curves)


def _encode(lat):
  """Lane-center lateral at each LOOKAHEAD (m, +left) -> 40 raw offsets. raw_i = -GAIN[i]*lat_i (stock = -OP lateral)."""
  raw = np.clip(np.round(-GAIN * np.asarray(lat, dtype=float)), -OFFSET_VALID_MAX, OFFSET_VALID_MAX)
  return [int(v) for v in raw]


def encode_lane_path(x, y):
  """OP lane center (x, y arrays in meters, +left) -> 40 raw offsets near->far (offline / arrays).

  Straight centered lane -> all zeros. Returns all-unavailable when OP has no usable lane reaching D_MAX.
  """
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  if x.size < 2 or x.max() < D_MAX:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  return _encode(np.interp(LOOKAHEAD, x, y))


def encode_lane_path_poly(poly, valid=True):
  """OP lane-center cubic coeffs [c0, c1, c2, c3] (y = c0 + c1*x + ..., +left) -> 40 raw offsets.

  This is the on-car path: the lane center is fit upstream (controlsd) and passed via carControlSP.
  Returns all-unavailable when `valid` is False or no coeffs are given.
  """
  if not valid or len(poly) == 0:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  return _encode(np.polyval(list(poly)[::-1], LOOKAHEAD))  # np.polyval takes highest-degree-first


def create_lane_path(packer, bus, offsets, mux):
  """Pack one LANE_PATH frame for MUX value `mux` (one of MUX_CYCLE) from the 40-offset array.

  The carcontroller cycles `mux` through all of MUX_CYCLE (40 values, 4 banks) at ~50 Hz; the
  logical index is (mux-1) % 16. COUNTER auto-increments and CHECKSUM (honda_checksum +10) is
  computed by the packer.
  """
  base = ((mux - 1) % 16) * OFFSETS_PER_INDEX
  values = {
    "MUX": mux,
    "PATH_OFFSET_1": offsets[base + 0],
    "PATH_OFFSET_2": offsets[base + 1],
    "PATH_OFFSET_3": offsets[base + 2],
    "PATH_OFFSET_4": offsets[base + 3],
  }
  return packer.make_can_msg("LANE_PATH", bus, values)


def create_lkas_hud_2(packer, bus, counter, reach=1.0):
  """Pack one LKAS_HUD_2 frame: enable both dash lane lines so OP's LANE_PATH renders.

  `reach` (0..1) scales LANE_LENGTH (the rendered dash reach): 1 = full length, shrinking toward 0
  retracts the lane far->near to fade it out on a model dropout instead of snapping off.

  CONSISTENCY (critical): the camera zeros ALL lane fields together when it has no lane. Sending
  "lanes enabled (LEFT/RIGHT=3, BOH=32) but LANE_LENGTH=0 / all-2047 path" is a frame the camera never
  emits, and the dash treats it as a fault -- it FREEZES the whole LKAS graphics layer (lanes AND the
  forwarded object icons) until a valid lane returns. So we gate LEFT/RIGHT_LANE and LKAS_BOH_1 on
  whether there's actually length to show, matching the camera's "no lane" frame.

  Replaces the camera's LKAS_HUD_2 (check_relay statically blocks it). At reach=1 it reproduces the
  camera's "lanes shown" frame byte-for-byte: both lines on, LANE_LENGTH maxed, LKAS_BOH_1 neutral
  (straight). The dash uses two rolling counters: COUNTER (covered by the checksum) and COUNTER_2,
  which trails COUNTER by one. CHECKSUM (honda_checksum +10, extended ID) is computed by the packer.
  """
  lane_length = max(0, min(LANE_LENGTH_MAX_VALUE, round(reach * LANE_LENGTH_MAX_VALUE)))
  shown = lane_length > 0   # no length -> zero every lane field (camera's "no lane" frame)
  values = {
    "COUNTER": counter,
    "COUNTER_2": (counter - 1) % 4,
    "SET_ME_X01": 1,
    "LKAS_BOH_1": LKAS_BOH_1_NEUTRAL if shown else 0,
    "LEFT_LANE": LANE_LINE_ON if shown else 0,
    "RIGHT_LANE": LANE_LINE_ON if shown else 0,
    "LEFT_LANE_CROSSED": 0,
    "RIGHT_LANE_CROSSED": 0,
    "LANE_LENGTH": lane_length,
  }
  return packer.make_can_msg("LKAS_HUD_2", bus, values)
