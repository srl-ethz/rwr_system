from enum import IntEnum, auto
from strenum import StrEnum

class HandChirality(StrEnum):
    LEFT = "left"
    RIGHT = "right"

class TeleopState(IntEnum):
    UNREADY = 0
    PAUSED = 1
    RUNNING = 2
