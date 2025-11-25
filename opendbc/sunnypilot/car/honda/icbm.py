"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CruiseButtons
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: CruiseButtons.RES_ACCEL,
  SendButtonState.decrease: CruiseButtons.DECEL_SET,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.current_button = SendButtonState.none
    self.button_send_remaining = 0
    self.button_pause_remaining = 0

  def update(self, CC_SP, packer, frame, last_button_frame, counter, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      if (self.button_send_remaining == 0 and
          self.button_pause_remaining == 0):
        self.current_button = self.ICBM.sendButton
        # Pulse button presses by sending then pausing
        self.button_send_remaining = 2
        self.button_pause_remaining = 6 # Pause long enough for HUD to update

       # Send button 1 frame before stock frame, which will block the stock frame from being forwarded
      if (self.frame - self.last_button_frame) % 4 == 2:
        if self.button_send_remaining > 0:
          send_button = BUTTONS[self.current_button]
          can_sends.append(hondacan.spam_buttons_command(packer, CAN, send_button, 0, self.CP.carFingerprint, counter=counter))
          self.button_send_remaining -= 1
        elif self.button_pause_remaining > 0:
          self.button_pause_remaining -= 1

    elif self.ICBM.sendButton == SendButtonState.none:
      self.button_send_remaining = 0
      self.button_pause_remaining = 0

    return can_sends
