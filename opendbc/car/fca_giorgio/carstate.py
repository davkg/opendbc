from opendbc.can.parser import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.fca_giorgio.values import DBC, CANBUS, CarControllerParams

GearShifter = structs.CarState.GearShifter

ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.CCP = CarControllerParams(CP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    # Button tracking
    # self.highway_assist_button = 0
    # self.acc_distance_button = 0
    # self.button_counter = 0


  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_FL"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_FR"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_RL"],
      pt_cp.vl["ABS_1"]["WHEEL_SPEED_RR"],
      unit=1.0,
    )
    ret.standstill = ret.vEgoRaw == 0

    ret.steeringAngleDeg = pt_cp.vl["EPS_1"]["STEERING_ANGLE"]
    ret.steeringRateDeg = pt_cp.vl["EPS_1"]["STEERING_RATE"]
    ret.steeringTorque = pt_cp.vl["EPS_2"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = pt_cp.vl["EPS_3"]["EPS_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > 100
    ret.yawRate = pt_cp.vl["ABS_2"]["YAW_RATE"]
    ret.steerFaultPermanent = bool(pt_cp.vl["EPS_2"]["LKA_FAULT"])

    gear = self.shifter_values.get(pt_cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(gear)

    ret.cruiseState.available = pt_cp.vl["ACC_1"]["CRUISE_STATUS"] in (1, 2, 3)
    ret.cruiseState.enabled = pt_cp.vl["ACC_1"]["CRUISE_STATUS"] in (2, 3)
    ret.cruiseState.speed = pt_cp.vl["ACC_1"]["HUD_SPEED"] * CV.KPH_TO_MS

    # TODO: rename ACCEL_PEDAL to throttle as both gas pedal and ACC affect it
    # We can infer gasPressed with throttle and cruiseState, but unsure if reliable
    # ret.gasPressed = pt_cp.vl["ENGINE_1"]["ACCEL_PEDAL"] > 0 and ret.cruiseState.available
    ret.gasPressed = False
    ret.brake = pt_cp.vl["ABS_4"]["BRAKE_PRESSURE"]
    ret.brakePressed = bool(pt_cp.vl["ABS_3"]["BRAKE_PEDAL_SWITCH"])
    #ret.parkingBrake = TODO

    ret.leftBlinker = bool(pt_cp.vl["BCM_1"]["LEFT_TURN_STALK"])
    ret.rightBlinker = bool(pt_cp.vl["BCM_1"]["RIGHT_TURN_STALK"])

    # prev_acc_distance_button = self.acc_distance_button
    # self.acc_distance_button = pt_cp.vl["ACC_BUTTON"]["ACC_DISTANCE"]
    # self.highway_assist_button = pt_cp.vl["ACC_BUTTON"]["HIGHWAY_ASSIST"]
    # self.button_counter = pt_cp.vl["ACC_BUTTON"]["COUNTER"]

    # ret.buttonEvents = create_button_events(self.acc_distance_button, prev_acc_distance_button, {1: ButtonType.gapAdjustCruise})

    # ret.espDisabled = TODO

    self.frame += 1
    return ret


  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.cam),
    }