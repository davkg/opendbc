# Render OpenPilot's lane onto the Honda Bosch radarless dash via the LANE_PATH + LKAS_HUD_2 messages.
#
# LANE_PATH is the camera's lane geometry as 40 lateral offsets (10 MUX indices x 4 sub-offsets,
# near->far), sent ~50 Hz cycling MUX over 4 redundant banks. LKAS_HUD_2 (5 Hz) is the dash's
# lane-display enable/length. We DON'T reproduce the camera's (smoothed, slow) path -- we feed OP's
# own, more responsive lane through the same wire format.
#
# Geometry: the 40 points are lateral offsets at fixed look-ahead distances. We split the lane center
# into two parts so each can be tuned (and later smoothed) independently:
#   - centering: the car's lateral position within the lane (the path's offset at the car, c0). It
#     shows up as a uniform shift on every point; CENTER_GAIN sets how strongly off-center reads.
#   - curvature: the lane shape ahead, relative to the near point. On a curve the lateral grows
#     ~0.5*k*d^2, so evenly spaced distances + a single CURVE_GAIN already give tiny near offsets and
#     large far ones -- no distance-dependent scale needed.
# Knobs (all dialed in on-device by watching the rendered lane):
#   D_MAX        -- how far ahead the points reach
#   CENTER_GAIN  -- raw units per meter of in-lane lateral offset (centering)
#   CURVE_GAIN   -- raw units per meter of lane curvature (shape ahead)
# The exact distance/scale can't be separated from logs (degenerate), but it doesn't need to be:
# pick a sane mapping, render, and calibrate live (sign is settled: stock offset = -OP lateral).
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
D_MAX = 80.0                                         # farthest point look-ahead (m)
# CENTER_GAIN: raw units per meter of in-lane offset (car's position within lane). A shared gain of 28
# over-displayed offset ~2x on the first drive (hugging the right line drew the car centered ON it), so ~half.
# CURVE_GAIN: raw units per meter of lane curvature (shape ahead, rel. to near point). If the dash scale is
# uniform the same ~2x exaggerates curves too -- left at 28 pending the deferred curve-smoothing pass.
CENTER_GAIN = 14.0
CURVE_GAIN = 28.0
LOOKAHEAD = np.linspace(D_NEAR, D_MAX, NUM_PTS)      # the 40 look-ahead distances, near->far

# LKAS_HUD_2 lane-display fields (camera values, decoded from logs). We always enable both lines.
LANE_LINE_ON = 3                                     # LEFT_LANE / RIGHT_LANE "shown" value (0 = off)
LANE_LENGTH_MAX_VALUE = 33                           # observed camera max; LANE_LENGTH scales the dash reach -> max = full
LKAS_BOH_1_NEUTRAL = 32                              # coarse road-curvature indicator, 32 = straight (deviates ~+-6 on curves)


def _encode(y0, lat_full):
  """In-lane offset y0 (m, +left) + lane-center lateral at each LOOKAHEAD (m, +left) -> 40 raw offsets.

  Centering (y0, a uniform shift) and curvature (lat_full - y0) get separate gains so each is tunable
  on its own. Stock offset = -OP lateral, hence the leading minus.
  """
  lat_full = np.asarray(lat_full, dtype=float)
  raw = -(CENTER_GAIN * y0 + CURVE_GAIN * (lat_full - y0))
  raw = np.clip(np.round(raw), -OFFSET_VALID_MAX, OFFSET_VALID_MAX)
  return [int(v) for v in raw]


def encode_lane_path(x, y):
  """OP lane center (x, y arrays in meters, +left) -> 40 raw offsets near->far (offline / arrays).

  Straight centered lane -> all zeros. Returns all-unavailable when OP has no usable lane reaching D_MAX.
  """
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  if x.size < 2 or x.max() < D_MAX:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  y0 = float(np.interp(0.0, x, y))                   # in-lane offset at the car (centering)
  return _encode(y0, np.interp(LOOKAHEAD, x, y))


def encode_lane_path_poly(poly, valid=True):
  """OP lane-center cubic coeffs [c0, c1, c2, c3] (y = c0 + c1*x + ..., +left) -> 40 raw offsets.

  This is the on-car path: the lane center is fit upstream (controlsd) and passed via carControlSP.
  c0 is the in-lane offset at the car (centering); the rest is curvature. Returns all-unavailable when
  `valid` is False or no coeffs are given.
  """
  if not valid or len(poly) == 0:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  poly = list(poly)
  return _encode(poly[0], np.polyval(poly[::-1], LOOKAHEAD))  # np.polyval takes highest-degree-first


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

  Replaces the camera's LKAS_HUD_2 (check_relay statically blocks it). At reach=1 it reproduces the
  camera's "lanes shown" frame byte-for-byte: both lines on, LANE_LENGTH maxed, LKAS_BOH_1 neutral
  (straight). The dash uses two rolling counters: COUNTER (covered by the checksum) and COUNTER_2,
  which trails COUNTER by one. CHECKSUM (honda_checksum +10, extended ID) is computed by the packer.
  """
  lane_length = max(0, min(LANE_LENGTH_MAX_VALUE, round(reach * LANE_LENGTH_MAX_VALUE)))
  values = {
    "COUNTER": counter,
    "COUNTER_2": (counter - 1) % 4,
    "SET_ME_X01": 1,
    "LKAS_BOH_1": LKAS_BOH_1_NEUTRAL,
    "LEFT_LANE": LANE_LINE_ON,
    "RIGHT_LANE": LANE_LINE_ON,
    "LEFT_LANE_CROSSED": 0,
    "RIGHT_LANE_CROSSED": 0,
    "LANE_LENGTH": lane_length,
  }
  return packer.make_can_msg("LKAS_HUD_2", bus, values)
