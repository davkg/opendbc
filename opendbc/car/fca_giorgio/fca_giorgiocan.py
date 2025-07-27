from opendbc.car.crc import CRC8J1850

def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "LKA_ACTIVE": lkas_enabled,
    "LKA_TORQUE": apply_steer,
  }

  return packer.make_can_msg("LKA_COMMAND", bus, values)


def create_lka_hud_1_control(packer, bus, lat_active):
  values = {
    "NEW_SIGNAL_5": 1,
    "NEW_SIGNAL_4": 6,
  }

  return packer.make_can_msg("LKA_HUD_1", bus, values)


def create_lka_hud_2_control(packer, bus, lat_active):
  values = {
    "NEW_SIGNAL_1": 1,
  }

  return packer.make_can_msg("LKA_HUD_2", bus, values)


def create_acc_button_control(packer, bus, counter, cancel_button=False):
  values = {
    "SPEED_UP": 1,  # Default state is 1
    "COUNTER": (counter + 1) % 16,
  }

  if cancel_button:
    values["CANCEL_OR_RADAR"] = 1

  return packer.make_can_msg("ACC_BUTTON", bus, values)


def fca_giorgio_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8J1850[crc]
  if address == 0xDE:
    return crc ^ 0x10
  elif address == 0x106:
    return crc ^ 0xF6
  elif address == 0x122:
    return crc ^ 0xF1
  elif address == 0x2FA:
    return crc ^ 0xBE
  else:
    return crc ^ 0x0A