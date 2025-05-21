# Abstract
The joystick_driver package hosts **"JoystickPublisher"  node** publishes joystick(8bitdo controller) input data to the /joystick_input topic

## Usage
```bash
ros2 run joystick_driver joystick_node 
```
## Structure
lx ly rx ry: [-32768  32767]

L2 R2: [0,255]

D-pad X Y: {-1, 0, 1}

a b x y: bool{True, False}

***NOTICCE!!! : ly ? ry????,????? ly:-1***

---
example:

lx: 0

ly: -1

rx: 0

ry: -1

dx: 0

dy: 0

l2: 0

r2: 0

a: false

b: false

x: false

y: false

l1: false

r1: false

l3: false

r3: false

select: false

start: false