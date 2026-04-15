Specifications of this FRC Team 801 repository (this is written by a human):



## **Project overview**

##### **Technical Specifications**



Our robot uses a ROBORIO 2.0 computer, and a fault-tolerant CAN bus architecture from the ROBORIO to peripheral motors and other devices. We uses CTRE swerves, specifically Kraken x60s for the main driving and x44s for the rotation of each wheel (in total, 4 motors of each type.) For the rest of the robot, REV NEO Vortex motors are used.



Our advanced design uses an extendable hopper with a gatherer (roller) at the end, allowing for high capacity and intake efficiency. The gatherer uses two NEO Vortex motors to spin the roller. The hopper uses one motor to extend and retract a rail system with a buffer; the system should be tuned so as not to damage the buffer. The rail uses the SparkFlex built-in motor encoder for closed-loop position control (units: motor rotations, zeroed on startup) with separate WPILib software PID gains for the extend and retract directions. A REV Through Bore Encoder (connected to roboRIO DIO ports 1 and 3) is kept initialized but unused. Full extension = 15.801 motor rotations. The robot cannot intake with the gatherer unless the hopper is extended, except while the jostling routine is active: in that case the extension guard is bypassed and the hopper oscillates under the extend PID between `kExtendedSetpoint` (15.801) and `kJostleSetpoint` (= `kExtendedSetpoint` − `kJostleAmplitude` = 11.801 motor rotations) to dislodge stuck game pieces. When the jostle command ends, the hopper parks at the fully extended setpoint.



The robot features a climb mechanism powered by a single NEO Vortex motor with closed-loop position control via a SparkFlex controller. The climb motor uses brake idle mode to prevent the robot from descending when not actively driven. Two positions are used: rest (encoder = 0) and fully climbed (configurable setpoint in motor rotations, zeroed on startup). When climbing is engaged, the turret subsystem also activates vertical displacement compensation to account for robot tilt and height changes.



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

* If not already extended (and not jostling), runs Hopper.extend(); then runs Gather.gather() at the default intake power

possessWithPower(power: double)

* Same extension guard as possess(), but spins the gatherer at the caller-supplied power (used for trigger-scaled intake)

setJostling(on: boolean)

* Enables/disables jostling mode. While true, the "hopper-must-be-extended-before-gathering" guard in possess()/possessWithPower() is bypassed so the hopper can oscillate freely within the jostle range even if gather calls are running.

jostleTo(position: double)

* Passthrough to Hopper.jostleTo() — commands the hopper to a jostle-range setpoint using the extend PID

isHopperAt(target: double, tol: double): boolean

* Passthrough to Hopper.isAt() — true when the motor encoder is within `tol` motor rotations of `target`

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

* Retracts the hopper toward its home position (setpoint = 2 motor rotations, within the configured retract band)

jostleTo(position: double)

* Drives to a jostle-range setpoint (motor rotations) using the extend PID. Both jostle endpoints live well above retract home, so the tuned extend gains are reused for both legs of the oscillation.

isAt(target: double, tol: double): boolean

* True when the motor encoder is within `tol` motor rotations of `target`

isExtended(): boolean

* True when the motor encoder is inside the configured extended band (`kExtendMinPosition`..`kExtendMaxPosition`)

isRetracted(): boolean

* True when the motor encoder is inside the configured retracted band (`kRetractMinPosition`..`kRetractMaxPosition`)

check(): boolean

* **Deprecated.** Use `isExtended()` instead; retained only for backwards compatibility.

stop()

* Disables the software PID loop and stops the motor immediately

testRun(power: double) / testSetPosition(position: double) / setTestMode(enabled: boolean)

* Test-mode hooks for raw-power driving, PID target tuning, and telemetry publishing to the `TestMode/Hopper` NetworkTables table



**climb.java (class Climb)**

extend()

* Drives the climb motor toward `kExtendSetpoint` under the climb PID (fully deployed position)

climb()

* Drives the climb motor toward `kClimbedSetpoint` under the climb PID (fully climbed position)

rest()

* Drives the climb motor toward encoder position 0 under the reset PID

stop() / testRun(power: double) / testSetPosition(position: double) / setTestMode(enabled: boolean)

* Immediate stop plus test-mode hooks analogous to Hopper



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
* For detailed turret mathematics (aim equations, vertical displacement compensation, lead compensation), see ABOUT5.md



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



Jostling(possession: Possession, hopper: Hopper)

* Sets `Possession.setJostling(true)` to disable the "hopper must be extended before gathering" guard, then oscillates the hopper PID target between `HopperConstants.kJostleSetpoint` (11.801) and `HopperConstants.kExtendedSetpoint` (15.801) under the standard extend PID
* Flips the target each time the motor encoder is within `kJostleTolerance` of the current target
* `isFinished()` always returns false — runs until interrupted (typically `whileTrue` on the Y button)
* On end, clears the jostling flag and parks the hopper at the fully extended setpoint
* Requires `Possession` and `Hopper`



Climbing(turret: TurretSubsystem, climb: Climb)

* Drives the climb motor toward the fully climbed setpoint via `Climb.climb()` and engages the turret's vertical displacement compensation while held
* Currently *not* bound in `RobotContainer` (the binding is commented out); D-pad Up/Down are wired directly to `Climb.extend()` / `Climb.rest()` instead


