from opendbc.can import CANPacker
from opendbc.car import Bus, apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fca_giorgio import fca_giorgiocan
from opendbc.car.fca_giorgio.values import CANBUS, CarControllerParams

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.frame = 0

    # Button tracking
    self.highway_assist_pressed_last = False
    self.acc_distance_pressed_last = False
    self.cancel_button_send_frame = -1
    self.cancel_button_end_frame = -1


  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # **** Button Test ****************************************************** #
    # Send cancel button 1 second after highway assist button is detected
    # This does not work, car doesn't respond to the cancel signal
    # TODO: Figure out why this doesn't work

    highway_assist_pressed = CS.highway_assist_button > 0

    # Detect rising edge of highway assist button
    if highway_assist_pressed and not self.highway_assist_pressed_last:
      self.cancel_button_send_frame = self.frame + 101 # 101 to land on 50Hz after 1 second
      self.cancel_button_end_frame = self.frame + 501 # Hold cancel for 5 seconds

    if (
      self.frame >= self.cancel_button_send_frame and
      (self.frame - self.cancel_button_send_frame) % 2 == 0 and # 50Hz
      self.frame < self.cancel_button_end_frame
    ):
      can_sends.append(fca_giorgiocan.create_acc_button_control(self.packer_pt, CANBUS.pt, CS.button_counter, cancel_button=True))
      can_sends.append(fca_giorgiocan.create_acc_button_control(self.packer_pt, 2, CS.button_counter, cancel_button=True))

    # self.highway_assist_pressed_last = highway_assist_pressed

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(fca_giorgiocan.create_steering_control(self.packer_pt, CANBUS.pt, apply_torque, CC.latActive))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.HUD_1_STEP == 0:
      can_sends.append(fca_giorgiocan.create_lka_hud_1_control(self.packer_pt, CANBUS.pt, CC.latActive))
    if self.frame % self.CCP.HUD_2_STEP == 0:
      can_sends.append(fca_giorgiocan.create_lka_hud_2_control(self.packer_pt, CANBUS.pt, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends