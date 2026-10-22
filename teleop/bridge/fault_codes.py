"""Manufacturer L20 motor bit flags; no clearing or control operations.

Source: linker-bot/linkerbot-python-sdk, commit
927290b8e27f9e2df3cc5b0a0776c36e2fb5175e,
src/linkerbot/hand/l20/fault.py (0x59..0x5D layout matches G20 driver).
The publication does not provide a firmware-by-firmware applicability table.
"""
SOURCE = ("https://github.com/linker-bot/linkerbot-python-sdk/blob/"
          "927290b8e27f9e2df3cc5b0a0776c36e2fb5175e/src/linkerbot/hand/l20/fault.py")
FLAGS = {
    1: "motor rotor lock", 2: "motor overcurrent", 4: "motor stall fault",
    8: "voltage abnormal", 16: "self-check abnormal", 32: "overtemperature",
    64: "soft rotor lock (software rotor lock)", 128: "motor communication abnormal",
}


def fault_names(code):
    if type(code) is not int or not 0 <= code <= 255:
        raise ValueError("Fault code must be an integer byte")
    return [name for flag, name in FLAGS.items() if code & flag] or ["no fault"]
