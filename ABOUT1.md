Specifications of this FRC Team 801 repository (this is written by a human):



## **Project overview**

##### **Technical Specifications**



Our robot uses a ROBORIO 2.0 computer, and a fault-tolerant CAN bus architecture from the ROBORIO to peripheral motors and other devices. We uses CTRE swerves, specifically Kraken x60s for the main driving and x44s for the rotation of each wheel (in total, 4 motors of each type.) For the rest of the robot, REV NEO Vortex motors are used.



Our advanced design uses an extendable hopper with a gatherer (roller) at the end, allowing for high capacity and intake efficiency. The gatherer itself uses a motor to spin the roller. The hopper also uses a motor to expand and retract the hopper. The robot cannot intake with the gatherer unless the hopper is extended. The hopper uses a rail system with a buffer, though ideally the system is tuned as to not cause damage to the buffer. This rail system only requires one motor. The rail system uses the SparkFlex built-in motor encoder for closed-loop position control (units: motor rotations, zeroed on startup). A REV Through Bore Encoder (connected to roboRIO DIO ports 1 and 3) is kept initialized but unused. Full extension = 27.183872 motor rotations; partial extension = 23.245955 motor rotations.



The centerpiece of our robot is a spindexer which stores and deposits game pieces to be later sent to the feeder. Its job is to quickly launch game pieces towards the feeder.



Our scoring mechanism consists of a feeder, one motor that powers a series of spinners on a horizontal-to-vertical ramp into the turret. The turret has a few motors. The first is connected to a REV Through Bore Encoder (connected to roboRIO DIO port 0) configured in absolute mode, allowing 420° of total turret travel (configured so 420° = 1 full encoder revolution). The turret has a final launching wheel that is powered by two motors that face each other (i.e. they must be configured so that one of them is reversed to work in tandem). For the launching system, it is likely that we'll be targeting a specific velocity, not a positional value. Finally, the turret has an angle manipulator, called a hood, that is designed to be able to change the trajectory of a piece anywhere from 15 to 45 degrees.




For most of the system, built-in motor encoders will be used, save the systems that simply spin the game pieces around without a target velocity, such as the gatherer. The spindexer and feeder use closed-loop velocity control targeting specific RPMs via SparkFlex PID, similar to the turret launch wheels.



##### **Software Specifications**



WPILib version 2026.2.1

QuestNav version 2026-1.0.0

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




**possession.java (class Possession)**

possess()

* If not already extended, runs Hopper.extend(); then runs Gather.gather() at the default intake power

stop()

* Stops the gatherer (hopper stays in current position)



**launch.java (class Launch)**

launch()

* Runs Spindex.spin()
* Runs Feeder.spin() at target velocity
* Runs Gather.gather() at positive half of kDefaultPower (forward direction, same as intake)

stop()

* Stops spindexer and feeder




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

retract()

* Retracts the hopper to its starting point (encoder = 0)

check(): boolean

* Returns whether the hopper is currently considered extended



**spindex.java (class Spindex)**

spin()

* Spins the spindexer at target velocity via closed-loop PID (toward feeder)

rest()

* Sets the spindexer motor power to 0

**feeder.java (class Feeder)**

spin()

* Spins the feeder at target velocity via closed-loop PID; positive = scoring direction

rest()

* Sets the feeder motor power to 0



**TurretSubsystem.java (class TurretSubsystem)**

* Self-contained auto-aiming subsystem that runs in periodic()
* Accepts QuestSubsystem via constructor for robot pose data
* Automatically calculates turret rotation, hood angle, and flywheel velocity based on distance to goal
* Goal coordinates are alliance-aware: blue goal at (4.635, 4.034), red goal at (11.9068, 4.034). Red is X-mirrored from blue; Y is preserved per standard FRC field mirroring.
* Uses PID + feedforward control for turret rotate, hood tilt, and shooter motors
* The turret automatically compensates for robot velocity by leading its rotation target, allowing scoring while moving at reduced drive speed
* Supports test mode (setTestMode/testRunLaunch/testRunHood/testRunRotate) for manual control



##### Commands



DriveToPose(drive: Drive, targetPose: Pose2d)

* Drives to a field-relative target pose using three profiled PID controllers (X, Y, rotation)
* Finishes when within 5 cm and 2° of target




Gathering(possession: Possession)

* Runs Possession.possess() each loop; stops on end



Shoot(launch: Launch)

* Runs Launch.launch() each loop; stops on end





Score(manipulator: Manipulator)

* Runs Manipulator.score() each loop; stops on end
* Primarily for autonomous use


