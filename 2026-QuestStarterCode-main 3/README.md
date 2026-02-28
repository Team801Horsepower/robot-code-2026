Command Robot Template — Porting & Tech Requirements
===============================================

This repository is a small, command-based WPILib robot project with a CTRE Phoenix 6 swerve drivetrain implementation (generated Tuner constants). The instructions below explain how to port the drive system to a new robot, required software/hardware, and recommended verification steps.

Quick summary
- Required files to keep for the drivetrain to work: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`, `src/main/java/frc/robot/generated/TunerConstants.java`, `src/main/java/frc/robot/RobotContainer.java`, `src/main/java/frc/robot/Constants.java`, `src/main/java/frc/robot/Robot.java`, `src/main/java/frc/robot/Main.java`.
- Minimal porting tasks: update ports/CAN IDs/encoder offsets/module positions in `TunerConstants` and hardware ports in `Constants.java`, confirm controller port in `RobotContainer`, set max speeds/limits and test carefully.

Table of contents
- Tech requirements
- Files to review and edit
- Step-by-step porting guide
- Testing and verification
- Safety and troubleshooting
- Optional cleanup & structure notes

Tech requirements
-----------------
Software
- Java 17 (FRC / WPILib runtime compatibility; this project was built against JRE 17).
- Gradle (the project includes the Gradle wrapper: use `./gradlew.bat` on Windows PowerShell).
- WPILib for the target season (use the same major release family the project was created with). Ensure you have the matching WPILib libraries and robot runtime on the RoboRIO.
- CTRE Phoenix 6 libraries (this project uses CTRE Phoenix 6 swerve APIs). Make sure vendordeps or Gradle dependencies include Phoenix 6.
- (Optional) PathPlanner and PathPlannerGUI if the repo uses `com.pathplanner.lib` auto builder (TunerConstants references PathPlanner config). Install PathPlanner on your machine to create paths.

Hardware
- Swerve modules (drive and steer motors, encoders) compatible with CTRE Phoenix 6 device classes used in `TunerConstants` (example uses TalonFX & CANcoder & Pigeon2).
- Properly wired CAN bus.
- Gyro/Pigeon2 or other IMU used by the drivetrain.
- A Windows/macOS/Linux host for development and a RoboRIO running a matching WPILib runtime.

Files to review and edit
------------------------
- `src/main/java/frc/robot/generated/TunerConstants.java`
  - This is the generated file with all drivetrain and module constants (CAN IDs, encoder offsets, module positions, gear ratios, simulation parameters, and factory method `createDrivetrain()`). Update these values to match your robot.

- `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`
  - Contains the drivetrain implementation (CTRE wrapper). You typically won't need to change this unless your hardware uses different APIs (different motor/encoder classes).

- `src/main/java/frc/robot/Constants.java`
  - Contains general robot constants such as controller ports. Update `OperatorConstants.kDriverControllerPort` and any other hardware mapping values.

- `src/main/java/frc/robot/RobotContainer.java`
  - Wires buttons, default commands, and creates the drivetrain via `TunerConstants.createDrivetrain()`. Update default speeds and controller mappings here to tune operator feel.

- `build.gradle` and `vendordeps/` (if present)
  - Ensure required vendor dependencies (Phoenix6, PathPlanner, etc.) are present. If you change CTRE device types or add external libraries, update build files accordingly.

Step-by-step porting guide
--------------------------
1. Clone/Copy this repository onto your development host.

2. Confirm the toolchain
- Install Java 17 JDK if you don't have it.
- Use the included Gradle wrapper: on Windows PowerShell from repository root:

```powershell
./gradlew.bat build --warning-mode=none
```

This verifies build and dependency resolution. If the build fails, fix dependency versions in `build.gradle`.

3. Update hardware mappings
- Open `TunerConstants.java` and update the following fields for each swerve module:
  - Drive motor CAN IDs and steer motor CAN IDs.
  - Azimuth encoder (CANcoder) ID and encoder offset Angle (this is crucial to align wheel zero positions).
  - Module X/Y positions relative to robot center (used for kinematics/odometry).
  - Pigeon/gyro ID (if used).
  - Motor inversion flags if your wiring or motors are physically inverted.
  - Gear ratios and wheel radius if your robot's hardware differs.

Note: Many of these fields are annotated or documented in the generated file; treat the encoder offsets especially carefully.

4. Update `Constants.java`
- Map driver controller port (USB port index visible in Driver Station) and any PWM/DIO/PCM/Power distribution ports used elsewhere.

5. Confirm drivetrain construction
- `TunerConstants.createDrivetrain()` returns a `CommandSwerveDrivetrain` instance configured with the constants. If your motor classes are different than the generated factory uses (e.g., you use different motor vendor), update the factory/constructor or use a compatible adapter.

6. Tune default controls in `RobotContainer.java`
- Set `kMaxSpeedMetersPerSecond` and `kMaxAngularRateRadPerSec` to safe defaults and reduce them for initial tests.
- Confirm controller axis mapping (left stick X/Y and triggers) match your controller.

7. Build and deploy
- Build the project:

```powershell
./gradlew.bat build --warning-mode=none
```

- Deploy to the RoboRIO (if using WPILib tasks) or copy the jar to your deployment target as your workflow requires.

8. Initial testing (hardware)
- Power the robot in a safe environment (e.g., wheels lifted off the ground or robot on blocks).
- Test basic steering: command the wheels to rotate slowly and verify encoder offsets and steer direction are correct.
- Test low-speed drive: set software limits (reduce kMaxSpeedMetersPerSecond to a small value like 0.5 m/s) and verify wheel directions.

9. Calibrate encoder offsets
- If wheels are not aligned when commanded to zero, adjust the encoder offsets in `TunerConstants` and re-deploy until the known zero positions match physical orientation. Many teams measure and set the offsets while the robot is physically in a known orientation.

10. Verify odometry & gyro
- Run simple straight-drive and rotation tests to confirm that odometry updates reflect real motion.

11. Run SysId (optional)
- The code includes SysId routines (accessible via controller combos). Use them to characterize translation/rotation if you plan to tune PID/feedforward gains.

Testing and verification checklist
---------------------------------
- [ ] Build succeeds locally.
- [ ] Robot boots and code runs on RoboRIO.
- [ ] Wheels rotate when you command low-speed forward/backward.
- [ ] Wheels steer to commanded angles (verify encoder offset correctness).
- [ ] Gyro/pigeon reports expected yaw values.
- [ ] Odometry pose roughly matches measured motion.
- [ ] Default command does not throw exceptions when running.

Safety notes
------------
- Always start with low speeds. Use `kMaxSpeedMetersPerSecond = 0.5` and a similarly low angular rate during initial hardware testing.
- Test with wheels off the ground or the robot on blocks whenever possible when first running new motor code.
- Ensure emergency stop procedures and disable switch are available and tested.
- Be careful with SysId routines: the provided code includes lower-voltage dynamic-step limits for safety, but still run SysId in a controlled environment.

Troubleshooting tips
--------------------
- Motor spinning the wrong direction: flip the inversion flag in `TunerConstants` for that motor or invert wiring.
- Wheel angles offset by a constant rotation: adjust the encoder offset Angle for that module in `TunerConstants`.
- NullPointer or missing class errors: make sure any optional bindings (vision, AprilTag align, Limelight helpers) that reference extra classes are removed or the associated class files are added.

Optional cleanup & structure notes
---------------------------------
- Generated files: `TunerConstants.java` is generated and large — prefer to keep it separate from hand-maintained files. If you regenerate it, avoid hand edits that will be overwritten.
- Example files: this project originally included example commands and subsystems; they were removed to create a minimal drivetrain-focused repo. If you want to add example autonomous routines, create new command classes under `src/main/java/frc/robot/commands/`.
- Combining files: you can combine small helper classes into fewer files for tiny projects, but keep subsystems and major components in their own files for clarity and maintainability.

Useful references
- CTRE Phoenix 6 docs: https://v6.docs.ctr-electronics.com/
- WPILib docs and Command-based guide: https://docs.wpilib.org/
- PathPlanner (if used): https://benbotfn.github.io/PathPlanner/

License
-------
This project follows the WPILib BSD license as included in the repository.

ps. the readme is ai generated so it may net be entirely accurate