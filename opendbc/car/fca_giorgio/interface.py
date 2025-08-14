from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.fca_giorgio.carcontroller import CarController
from opendbc.car.fca_giorgio.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "fca_giorgio"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fcaGiorgio)]
    ret.radarUnavailable = True

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    return ret