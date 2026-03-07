Specifications of this FRC Team 801 repository (this is written by a human):



## **Project overview**

##### **Technical Specifications**



Our robot uses a ROBORIO 2.0 computer, and a fault-tolerant CAN bus architecture from the ROBORIO to peripheral motors and other devices. We uses CTRE swerves, specifically Kraken x60s for the main driving and x44s for the rotation of each wheel (in total, 4 motors of each type.) For the rest of the robot, REV NEO Vortex motors are used.



Our advanced design uses an extendable hopper with a gatherer (roller) at the end, allowing for high capacity and intake efficiency. The gatherer itself uses a motor to spin the roller. The hopper also uses a motor to expand and retract the hopper. The robot cannot intake with the gatherer unless the hopper is extended. The hopper uses a rail system with a buffer, though ideally the system is tuned as to not cause damage to the bugger. This rail system only requires one motor. The rail system also has a REV V2 Absolute Encoder. However, this encoder ill be set in relative encoder mode, acting like a quadrature encoder.



The centerpiece of our robot is a spindexer which stores and deposits game pieces to be later sent to the feeder. Its job is twofold: it must quickly launch game pieces towards the feeder, but before that it must also move ever so slightly back and forth as an agitator. For agitation, a sinusoidal function could be used to (not too abruptly) sway the spindexer back and forth. A configuration option could be used to set the agitation type (flat, sinusoidal, absolute value \[repeating the absolute value equation every 2x using an interval from (-x, x) where f(-x) = f(x))



Our scoring mechanism consists of a feeder, one motor that powers a series of spinners on a horizontal-to-vertical ramp into the turret. The turret has a few motors. The first is connected to another REV V2 Absolute Encoder which can spin the turret 210 degrees in either direction, for a total capability of 420 degrees total. The absolute encoder would have to be configured where 420 degrees = 360 degrees for the encoder. The turret has a final launching wheel that is powered by two motors that face each other (i.e. they must be configured so that one of them is reversed to work in tandem). For the launching system, it is likely that we'll be targeting a specific velocity, not a positional value. Finally, the turret has an angle manipulator, called a hood, that is designed to be able to change the trajectory of a piece anywhere from 15 to 30 degrees. On the turret, there is a camera powered by PhotonVision.



The climber is a one-motor two-stage slide. It will need to be tuned to move from one position to the next, and not overshoot.



For most of the system, built-in motor encoders will be used, save the systems that simply spin the game pieces around without a target velocity, such as the gatherer, spindexer, and feeder.



##### **Software Specifications**



WPILib version 2026.2.1

QuestNav version 2026-2.1.0

Java version 17.0.12

... I'm sure there's others; when in doubt it's the latest version.



## **Hardware Map**

#### **Subsystem Methods**



This is by no means a comprehensive list. Instead, treat this as the core of what the robot should do, and fill in the gaps in the execution. Add wrappers as needed. Add parameters to methods as needed.



**manipulator.java (class Manipulator)**

score()

* Calls Possession.possess() and Launch.launch()
* Primarily for autonomous usage

stop()

* Stops possession and launch mechanisms



**drive.java (class Drive)**

applyRequest(requestSupplier: Supplier\<SwerveRequest\>): Command

* Returns a command that continuously applies the given swerve request each loop

seedPose(pose: Pose2d)

* Seeds the drivetrain pose estimator with a known pose (e.g. from QuestNav)

seedFieldRelative()

* Seeds field-centric heading so current robot direction becomes "forward"

getPose2d(): Pose2d

* Returns the latest estimated robot pose

getDrivetrain(): DrivetrainSubsystem

* Returns the underlying DrivetrainSubsystem for subsystems needing direct access



**vision.java (class Vision)**

report(): Pose2d

* Runs functions found in the child questnav and photon subsystems to report as much positional data as possible
* Returns the latest robot pose from the drivetrain's pose estimator

isQuestNavTracking(): boolean

* Returns whether QuestNav is currently tracking



**ascension.java (class Ascension)**

levelOne()

* Calls Level1.ascend()
* Sets a position variable to 1, denoting that the robot is currently at a level 1 climb

levelThree()

* Void function for now

descend()

* Calls Level1.descend() if the position variable is 1, then resets position to 0 once retracted

isAtSetpoint(): boolean

* Returns true when the Level 1 climber has physically reached its extended setpoint

getPosition(): int

* Returns the current climb level (0 = grounded, 1 = L1)



**possession.java (class Possession)**

possess()

* If not already extended, runs Hopper.extend(); then runs Gather.gather() at the default intake power

stop()

* Stops the gatherer (hopper stays in current position)



**launch.java (class Launch)**

launch()

* Runs Spindex.spin()
* Runs Feeder.spin() at full positive power (1.0)
* Runs Turret.spin() to keep launch wheels at velocity

stop()

* Stops feeder and spindexer (turret wheels keep spinning intentionally)



**level1.java (class Level1)**

ascend()

* Runs the climber motor at kMotorPower until the encoder reaches kExtendedSetpoint
* Once at setpoint, applies elevator feedforward voltage to hold against gravity

descend()

* Runs the climber motor in reverse until the encoder reaches roughly 0

isAtSetpoint(): boolean

* Returns true when the encoder is within tolerance of kExtendedSetpoint

isRetracted(): boolean

* Returns true when the encoder is at or below tolerance (fully retracted)



**gather.java (class Gather)**

gather(power: double)

* Runs the gatherer motor at the given power; positive = intake direction

rest()

* Stops spinning gatherer motor



**hopper.java (class Hopper)**

extend()

* Extends the hopper fully to kExtendedSetpoint via closed-loop position control

extendTo(pct: double)

* Extends the hopper to a percentage of full travel (0 = retracted, 100 = fully extended)

jostle()

* Oscillates the hopper between (1−amplitude)·setpoint and (1−2·amplitude)·setpoint
* Uses independent HopperConstants jostle constants (kJostleAgitationType, kJostleAmplitude, kJostlePeriod, kJostleReversed)

retract()

* Retracts the hopper to its starting point (encoder = 0)

check(): boolean

* Returns whether the hopper is currently considered extended



**spindex.java (class Spindex)**

spin()

* Spins the spindexer motor at negative the power stored in kSpinPower (toward feeder)

rest()

* Sets the spindexer motor power to 0

agitate()

* Applies a time-varying waveform (flat, sinusoidal, or absolute value/triangle) to prevent jams
* Waveform shape, amplitude, period, and center point (reversed flag) are set via constants



**feeder.java (class Feeder)**

spin(power: double)

* Spins the feeder motor at the given power in \[-1, 1\]; positive = scoring direction

rest()

* Sets the feeder motor power to 0



**turret.java (class Turret)**

spin()

* Uses closed-loop velocity control to run both launch motors at kTargetVelocityRPM
* Motors are inverted relative to each other to spin in tandem; no rest() — turret spins all match

aim(angleDegrees: double)

* Sets the hood angle to angleDegrees, clamped to \[kHoodMinDeg, kHoodMaxDeg\] (15–30°)
* Uses built-in relative encoder to drive the hood motor at kAimPower in the correct direction

rotate(angleDegrees: double)

* Rotates the turret to angleDegrees relative to robot forward, clamped to ±210°
* Uses REV V2 Absolute Encoder (1 revolution = 420° of turret travel); drives at kRotatePower

getRotationDeg(): double

* Returns the current turret rotation angle relative to robot forward (degrees)

getHoodEncoderPos(): double

* Returns the current hood encoder position (rotations from minimum angle)



##### Commands



DriveToPose(drive: Drive, targetPose: Pose2d)

* Drives to a field-relative target pose using three profiled PID controllers (X, Y, rotation)
* Finishes when within 5 cm and 2° of target



RunLaunchWheel(turret: Turret)

* Runs Turret.spin() each loop; used as the Turret subsystem's default command



ClimbL1(hopper: Hopper, ascension: Ascension)

* Runs Hopper.retract() and Ascension.levelOne() each loop
* Finishes when Ascension.isAtSetpoint() returns true



Gathering(possession: Possession)

* Runs Possession.possess() each loop; stops on end



Shoot(launch: Launch, hopper: Hopper)

* Runs Launch.launch() and Hopper.jostle() each loop; stops on end



Jostling(spindex: Spindex)

* Runs Spindex.agitate() each loop (hopper jostling removed)



Aim(vision: Vision, turret: Turret)

* Gets Vision.report() for current robot pose
* Computes bearing from robot to target field position and hood angle from distance
* Runs Turret.spin(), Turret.rotate(), and Turret.aim() each loop



Score(manipulator: Manipulator)

* Runs Manipulator.score() each loop; stops on end
* Primarily for autonomous use


