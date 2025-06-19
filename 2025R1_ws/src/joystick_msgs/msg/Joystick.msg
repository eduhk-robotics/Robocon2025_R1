# Joystick.msg

# Axes (analog inputs)
# Range typically -32768 to 32767 for sticks, 0 to 255 for triggers
int32 lx
int32 ly
int32 rx
int32 ry
int32 dx  # D-pad X (often -1, 0, 1)
int32 dy  # D-pad Y (often -1, 0, 1)
int32 l2  # Left Trigger
int32 r2  # Right Trigger

# Buttons (digital inputs)
bool a
bool b
bool x
bool y
bool l1
bool r1
bool l3  # Left stick press
bool r3  # Right stick press
bool select # minus
bool start  # plus
