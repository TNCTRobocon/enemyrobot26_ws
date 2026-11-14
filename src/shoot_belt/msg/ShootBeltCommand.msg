# COMMAND_CURRENT value is current in amperes (A).
# COMMAND_POSITION value is a signed position in millimeters (mm), relative to the most recent origin reset.
# COMMAND_MODE value is the numeric control-mode selector.
# COMMAND_ORIGIN_RESET transmits true to STM32 to reset its current position to 0 mm.
uint8 COMMAND_CURRENT=0
uint8 COMMAND_POSITION=1
uint8 COMMAND_MODE=2
uint8 COMMAND_ORIGIN_RESET=3

uint8 command_type
float32 value
