"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.honda.values import (HONDA_BOSCH, HONDA_BOSCH_RADARLESS, HONDA_BOSCH_CANFD)
from opendbc.sunnypilot.car.honda.hud_objects import HudObjectTracker
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP
from opendbc.car.common.conversions import Conversions as CV

# CAMERA_LEAD.LEAD_DISTANCE is a dedicated single-lead distance in centimeters, with an all-ones
# (65535) sentinel when the camera reports no lead. More accurate at range than the per-object
# LONG_DIST (tuned for the dash display); 0 means the message hasn't been received yet.
CAMERA_LEAD_NO_LEAD = 65535
CM_TO_M = 0.01


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    # Honda Bosch radarless camera publishes HUD_OBJECTS — up to 10
    # adjacent-vehicle tracks per cycle. Aggregate them for downstream consumers.
    self.hud_object_tracker = HudObjectTracker() if CP.carFingerprint in HONDA_BOSCH_RADARLESS else None

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP_SP.flags & HondaFlagsSP.HAS_CAMERA_MESSAGES:
      speed_bus = cp if (self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS - HONDA_BOSCH_CANFD)) else cp_cam
      speed_limit_raw = speed_bus.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"] % 32
      ret_sp.speedLimit = speed_limit_raw * 5.0 * CV.MPH_TO_MS if (1 <= speed_limit_raw <= 17) else 0.0

    if self.hud_object_tracker is not None:
      self.hud_object_tracker.update(cp_cam)
      # Dedicated single-lead distance (cm; 65535 = no lead), updated faster than
      # the object tracks. radard fuses it into the vision lead's speed estimate.
      raw_lead = cp_cam.vl["CAMERA_LEAD"]["LEAD_DISTANCE"]
      lead_valid = 0 < raw_lead < CAMERA_LEAD_NO_LEAD
      ret_sp.cameraLeadDistance = float(raw_lead) * CM_TO_M if lead_valid else 0.0
      ret_sp.cameraLeadValid = lead_valid
      ret_sp.hudObjects = [structs.CarStateSP.HudObject(slot=t.slot, objectId=t.object_id, dRel=t.d_rel,
                                                        yRel=t.y_rel, valid=t.valid, isLeadCar=t.is_lead_car)
                           for t in self.hud_object_tracker.snapshot()]

    if self.CP_SP.flags & HondaFlagsSP.NIDEC_HYBRID:
      ret.accFaulted = bool(cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_1"] or cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_2"])
      ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE_HYBRID"] > 1e-5)
      ret.blockPcmEnable = ret.brakeHoldActive # Nidec Hybrids fault if resuming cruise from brake hold

    if self.CP_SP.flags & HondaFlagsSP.HYBRID_ALT_BRAKEHOLD:
      ret.brakeHoldActive = cp.vl["BRAKE_HOLD_HYBRID_ALT"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP_SP.enableGasInterceptor:
      # Same threshold as panda, equivalent to 1e-5 with previous DBC scaling
      gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = gas > 492
