# Render OpenPilot's lane onto the Honda Bosch radarless dash via the LANE_PATH message.
#
# LANE_PATH is the camera's lane geometry as 40 lateral offsets (10 MUX indices x 4 sub-offsets,
# near->far), sent ~50 Hz cycling MUX over 4 redundant banks. We DON'T reproduce the camera's
# (smoothed, slow) path -- we feed OP's own, more responsive lane through the same wire format.
#
# Geometry: the 40 points are lateral offsets at fixed look-ahead distances. On any curve the
# lateral grows ~0.5*k*d^2, so evenly spaced distances + a single GAIN already yield tiny near
# offsets and large far ones -- no distance-dependent scale needed. That leaves two knobs:
#   D_MAX  -- how far ahead the points reach
#   GAIN   -- raw units per meter of lateral
# both dialed in on-device by watching the dash (use the test patterns below). The exact
# distance/scale can't be separated from logs (degenerate), but it doesn't need to be: pick a
# sane mapping, render, and calibrate live.
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

# --- the two tunable knobs (calibrate on-device) ---
D_NEAR = 2.0                                         # nearest point look-ahead (m)
D_MAX = 80.0                                         # farthest point look-ahead (m)
GAIN = 28.0                                          # raw units per meter of lateral (+left lateral -> negative raw)
LOOKAHEAD = np.linspace(D_NEAR, D_MAX, NUM_PTS)      # the 40 look-ahead distances, near->far

# LANE_LENGTH (LKAS_HUD_2): speed-scaled dash reach, ~0.73*vEgo + 8 (empirical); 0 when no lane.
LANE_LENGTH_PER_MS = 0.73
LANE_LENGTH_BASE = 8.0
LANE_LENGTH_MIN = 5.0                                # floor at standstill while a lane is shown
LANE_LENGTH_MAX = 35.0                               # stay within the camera's observed range


def _offsets_from_lateral(lat):
  """Lateral (m, +left) at each LOOKAHEAD -> 40 clamped raw offsets. raw = clip(-GAIN * lat)."""
  raw = np.clip(np.round(-GAIN * np.asarray(lat, dtype=float)), -OFFSET_VALID_MAX, OFFSET_VALID_MAX)
  return [int(v) for v in raw]


def encode_lane_path(x, y):
  """OP lane center (x, y arrays in meters, +left) -> 40 raw offsets near->far (offline / arrays).

  Straight lane -> all zeros. Returns all-unavailable when OP has no usable lane reaching D_MAX.
  """
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  if x.size < 2 or x.max() < D_MAX:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  return _offsets_from_lateral(np.interp(LOOKAHEAD, x, y))


def encode_lane_path_poly(poly, valid=True):
  """OP lane-center cubic coeffs [c0, c1, c2, c3] (y = c0 + c1*x + ..., +left) -> 40 raw offsets.

  This is the on-car path: the lane center is fit upstream (controlsd) and passed via carControlSP.
  Returns all-unavailable when `valid` is False or no coeffs are given.
  """
  if not valid or len(poly) == 0:
    return [OFFSET_UNAVAILABLE] * NUM_PTS
  lat = np.polyval(list(poly)[::-1], LOOKAHEAD)  # np.polyval takes highest-degree-first
  return _offsets_from_lateral(lat)


def lane_length(v_ego, has_lane):
  """LANE_LENGTH value for LKAS_HUD_2: speed-scaled dash reach, 0 when no lane is shown."""
  if not has_lane:
    return 0
  return int(np.clip(LANE_LENGTH_PER_MS * v_ego + LANE_LENGTH_BASE, LANE_LENGTH_MIN, LANE_LENGTH_MAX))


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
