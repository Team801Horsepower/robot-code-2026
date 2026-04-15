# Controls



The robot will be operated by one (1) standard Xbox Controller. Below is the correct mapping to functions:

Left Trigger: Intake. Where the button is pressed more than 0.08, scale the power of the gather motor accordingly (where 0.5 = 0.5 power)
Left Bumper: Reverse intake. Full power, but make the power configurable
Left Joystick (x and y axes): Translation of our robot
Right Trigger: Launch, where the button is pressed more than 0.15, begin the launch sequence. The robot's drive speed is also reduced when pressed more than 0.15 to allow scoring while moving.
Right Joystick (x axis): Rotate robot
X Button: Retract hopper toward home position (use with caution — hopper retract risks damage to the buffer)
B Button: Reverse spindexer while held
Y Button: Jostle hopper — oscillates the hopper between `kExtendedSetpoint` (15.801) and `kJostleSetpoint` (11.801) under the extend PID to dislodge stuck game pieces. While held, the normal "hopper must be extended before gathering" guard in Possession is bypassed. Releasing the button parks the hopper fully extended.
D-pad Up: Drive climb motor to its fully extended position (`Climb.extend()`)
D-pad Down: Drive climb motor toward rest position (`Climb.rest()`)

Note: the full `Climbing` command (which also engages the turret's vertical displacement compensation) exists in the codebase but is currently not bound; D-pad Up/Down call the Climb subsystem methods directly.