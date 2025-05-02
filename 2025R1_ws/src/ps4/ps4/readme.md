PS4 Controller ROS 2 Package
This ROS 2 package allows you to interface with a PS4 controller, publishing its data to a topic and subscribing to that topic to print the data. The package consists of two nodes: ps4_publisher and ps4_subscriber.

Nodes
ps4_publisher
This node reads all the button and joystick data from the PS4 controller and publishes the data under the topic ps4.

ps4_subscriber
This node subscribes to the ps4 topic and prints the data received from the PS4 controller.

Data Ranges
The data from the PS4 controller is sent in a sensor_msgs/msg/Joy message, which includes two arrays: axes and buttons.

Axes Array
The axes array contains floating-point values representing the positions of the analog sticks and triggers. The typical data ranges and their corresponding indices are as follows:

Left Analog Stick Horizontal Axis: axes[0] (-1 to 1)
Left Analog Stick Vertical Axis: axes[1] (-1 to 1)
Right Analog Stick Horizontal Axis: axes[2] (-1 to 1)
Right Analog Stick Vertical Axis: axes[3] (-1 to 1)
L2 Trigger: axes[4] (-1 to 1)
R2 Trigger: axes[5] (-1 to 1)
Buttons Array
The buttons array contains integer values representing the state of the buttons (pressed or not pressed). The typical indices and their values are as follows:

Square Button: buttons[0] (0/1)
Cross Button (X): buttons[1] (0/1)
Circle Button: buttons[2] (0/1)
Triangle Button: buttons[3] (0/1)
L1 Button: buttons[4] (0/1)
R1 Button: buttons[5] (0/1)
L2 Button: buttons[6] (0/1)
R2 Button: buttons[7] (0/1)
Share Button: buttons[8] (0/1)
Options Button: buttons[9] (0/1)
PS Button: buttons[10] (0/1)
Touchpad Button: buttons[11] (0/1)
Left Stick Button (L3): buttons[12] (0/1)
Right Stick Button (R3): buttons[13] (0/1)