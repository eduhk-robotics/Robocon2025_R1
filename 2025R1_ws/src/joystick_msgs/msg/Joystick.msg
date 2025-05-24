# Joystick.msg

# Analog axes
int32 lx
int32 ly
int32 rx
int32 ry
int32 cx  # D-pad X (often treated as axis by evdev)
int32 cy  # D-pad Y (often treated as axis by evdev)
int32 lt  # Left trigger
int32 rt  # Right trigger

# Digital buttons
bool a
bool b
bool x
bool y
bool lb     # Left bumper
bool rb     # Right bumper
bool l3     # Left stick press
bool r3     # Right stick press
bool minus  # Often 'select' or 'back'
bool plus   # Often 'start'

