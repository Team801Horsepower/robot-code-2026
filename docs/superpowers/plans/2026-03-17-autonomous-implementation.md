# Autonomous Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire PathPlanner autos to the CTRE swerve drivetrain, add alliance-aware field zones for hood management, and seed QuestNav at auto start.

**Architecture:** PathPlanner's AutoBuilder drives the CTRE swerve via `ApplyRobotSpeeds`. Field zone logic in `TurretSubsystem.periodic()` reads QuestNav pose and alliance color to control hood behavior globally. QuestNav is seeded at auto start from known starting positions.

**Tech Stack:** Java 17, WPILib 2026.2.1, CTRE Phoenix 6, PathPlanner (pathplannerlib), QuestNav 2026-2.1.0

**Spec:** `docs/superpowers/specs/2026-03-17-autonomous-implementation-design.md`

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `vendordeps/PathplannerLib.json` | Create | PathPlanner vendordep |
| `src/main/java/frc/robot/Constants.java` | Modify | Add `FieldConstants`, update `RedGoal` values, add `AutoConstants` with starting poses |
| `src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java` | Modify | Add `getChassisSpeeds()` method |
| `src/main/java/frc/robot/subsystems/QuestSubsystem.java` | Modify | Add public `setPose(Pose3d)` method |
| `src/main/java/frc/robot/subsystems/TurretSubsystem.java` | Modify | Alliance-aware aiming + field zone hood logic |
| `src/main/java/frc/robot/RobotContainer.java` | Modify | AutoBuilder config, auto chooser, NamedCommands, `seedAutoStartPose()` |
| `src/main/java/frc/robot/Robot.java` | Modify | Call `seedAutoStartPose()` in `autonomousInit()` |
| `ABOUT4.md` | Create | Autonomous paths documentation |
| `ABOUT1.md` | Modify | Red goal mirroring note |

---

## Chunk 1: Prerequisites and Constants

### Task 1: Install PathPlanner Vendordep

**Files:**
- Create: `vendordeps/PathplannerLib.json`

- [ ] **Step 1: Install PathPlanner library**

Run the WPILib "Manage Vendor Libraries" command in VS Code, or manually download the vendordep JSON. The 2026 PathPlanner vendordep URL is:
```
https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
```

Place it in `vendordeps/PathplannerLib.json`.

- [ ] **Step 2: Verify it compiles**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL (PathPlanner classes now available on classpath)

- [ ] **Step 3: Commit**

```bash
git add vendordeps/PathplannerLib.json
git commit -m "Add PathPlanner vendordep"
```

---

### Task 2: Add FieldConstants and Update RedGoal in Constants.java

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java`

- [ ] **Step 1: Add FieldConstants inner class with zone definitions and FieldZone enum**

Add before the closing `}` of `Constants.java` (after `TurretSubsystemConstants`):

```java
  // ─── Field ──────────────────────────────────────────────────────────────────

  public enum FieldZone { LAUNCH, TRENCH, FAR }

  public static final class FieldConstants {
    /** Field length in meters (54 ft 1 in). */
    public static final double kFieldLengthMeters = 16.5418;
    /** Field width in meters (26 ft 7.25 in). */
    public static final double kFieldWidthMeters = 8.0518;

    /** End of launch zone, measured from friendly wall (156.61 inches). */
    public static final double kLaunchZoneEndMeters = 3.978;
    /** End of trench zone, measured from friendly wall (201.01 inches). */
    public static final double kTrenchZoneEndMeters = 5.106;

    /**
     * Returns the field zone the robot is currently in.
     *
     * @param robotX Robot X position in meters (WPILib field coordinates)
     * @param alliance Current alliance color
     * @return The field zone
     */
    public static FieldZone getFieldZone(double robotX, edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
      double distFromFriendlyWall;
      if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
        distFromFriendlyWall = kFieldLengthMeters - robotX;
      } else {
        distFromFriendlyWall = robotX;
      }

      if (distFromFriendlyWall <= kLaunchZoneEndMeters) {
        return FieldZone.LAUNCH;
      } else if (distFromFriendlyWall <= kTrenchZoneEndMeters) {
        return FieldZone.TRENCH;
      } else {
        return FieldZone.FAR;
      }
    }
  }
```

- [ ] **Step 2: Update Red Alliance Goal Position constants**

In `TurretSubsystemConstants`, replace the red goal placeholder values:

```java
    // Red Alliance Goal Position (field-mirrored from blue; only X flipped)
    public static final double RedGoalX = 11.9068;
    public static final double RedGoalY = 4.034;
    public static final double RedGoalZ = 1.0;
```

- [ ] **Step 3: Add AutoConstants with starting poses**

Add after `FieldConstants`:

```java
  // ─── Autonomous ─────────────────────────────────────────────────────────────

  public static final class AutoConstants {
    // Starting positions extracted from PathPlanner path first waypoint anchors.
    // These are robot-center poses (PathPlanner uses robot-center coordinates).

    // Center start (linked name "Center" in C-H1.path, C-D1.path)
    public static final double kCenterStartX = 3.586;
    public static final double kCenterStartY = 4.067;
    public static final double kCenterStartYaw = Math.PI; // 180 degrees

    // Right start (linked name "Right" in R-S1.path, R-C1.path)
    public static final double kRightStartX = 3.648;
    public static final double kRightStartY = 2.210;
    public static final double kRightStartYaw = 0.0;

    // Left start (linked name "Left" in L-S1.path)
    public static final double kLeftStartX = 3.637;
    public static final double kLeftStartY = 5.794;
    public static final double kLeftStartYaw = Math.PI / 2.0; // 90 degrees

    // Far Right start (linked name "Far Right" in FR-S1.path)
    public static final double kFarRightStartX = 4.414;
    public static final double kFarRightStartY = 0.432;
    public static final double kFarRightStartYaw = 0.0;

    // Far Left start (linked name "Far Left" in FL-S1.path)
    public static final double kFarLeftStartX = 4.401;
    public static final double kFarLeftStartY = 7.625;
    public static final double kFarLeftStartYaw = 0.0;
  }
```

- [ ] **Step 4: Verify it compiles**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/Constants.java
git commit -m "Add FieldConstants, AutoConstants, and update red goal values"
```

---

## Chunk 2: Subsystem Modifications

### Task 3: Add getChassisSpeeds() to DrivetrainSubsystem

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java`

- [ ] **Step 1: Add getChassisSpeeds() method**

Add to the "Public API" section of `DrivetrainSubsystem.java` (after the `seedPose` method, around line 212):

```java
  /**
   * Returns the current robot-relative chassis speeds.
   *
   * <p>Required by PathPlanner's AutoBuilder for path following.
   *
   * @return Robot-relative ChassisSpeeds (vx forward, vy left, omega CCW)
   */
  public edu.wpi.first.math.kinematics.ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }
```

**Note:** CTRE Phoenix 6 uses PascalCase for `SwerveDriveState` fields (e.g., `Pose`, `ModuleStates`). The field should be `Speeds`. If this doesn't compile, check the CTRE Phoenix 6 javadoc for `SwerveDriveState` — it may be `speeds` or `RobotRelativeSpeeds` depending on the exact version. The build step will catch this immediately.

- [ ] **Step 2: Verify it compiles**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java
git commit -m "Add getChassisSpeeds() for PathPlanner AutoBuilder"
```

---

### Task 4: Add setPose() to QuestSubsystem

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/QuestSubsystem.java`

- [ ] **Step 1: Add public setPose and isTracking methods**

Add after the constructor (after line 47):

```java
  /**
   * Seeds QuestNav with a known robot pose. Call at the start of autonomous
   * to tell QuestNav where the robot has been placed on the field.
   *
   * @param pose The robot's known starting pose
   */
  public void setPose(Pose3d pose) {
    questNav.setPose(pose);
  }

  /** Returns true if QuestNav is actively tracking the headset pose. */
  public boolean isTracking() {
    return questNav.isTracking();
  }
```

- [ ] **Step 2: Verify it compiles**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/QuestSubsystem.java
git commit -m "Add public setPose() for auto start seeding"
```

---

### Task 5: Add Alliance-Aware Aiming and Zone Logic to TurretSubsystem

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/TurretSubsystem.java`

This is the most complex task. The turret needs:
1. A cached alliance color (set externally, not queried in periodic)
2. Goal coordinate selection based on alliance
3. Zone-based hood behavior

- [ ] **Step 1: Add alliance field and setter**

Add instance fields after `private boolean m_testMode = false;` (line 80):

```java
  private edu.wpi.first.wpilibj.DriverStation.Alliance m_alliance =
      edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

  /**
   * Caches the current alliance color. Call from autonomousInit/teleopInit.
   * Defaults to Blue if not set.
   */
  public void setAlliance(edu.wpi.first.wpilibj.DriverStation.Alliance alliance) {
    m_alliance = alliance;
  }
```

- [ ] **Step 2: Replace hardcoded blue goal with alliance-aware goal selection**

In `periodic()`, replace lines 120-121:
```java
    vX = Constants.TurretSubsystemConstants.BlueGoalX - TurretX;
    vY = Constants.TurretSubsystemConstants.BlueGoalY - TurretY;
```

With:
```java
    double goalX, goalY;
    if (m_alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
      goalX = Constants.TurretSubsystemConstants.RedGoalX;
      goalY = Constants.TurretSubsystemConstants.RedGoalY;
    } else {
      goalX = Constants.TurretSubsystemConstants.BlueGoalX;
      goalY = Constants.TurretSubsystemConstants.BlueGoalY;
    }
    vX = goalX - TurretX;
    vY = goalY - TurretY;
```

- [ ] **Step 3: Add zone-based hood logic**

In `periodic()`, wrap the existing hood control block (lines 142-159) with zone logic. Replace:

```java
    /*
     * TURRET HOOD
     * Takes the encoder value from the Spark Flex built in relative encoder and converts it to hood position in radians.
     * Calculates the hood theta target using the line of best fit from an analysis of several physics equations.
     * Feedforward helps to overcome system resistance.
     * Feedback drives turret hood motor to target hood theta.
     */
    HoodEncoder = s_HoodTiltMotor.getEncoder();
    HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;
    HoodThetaTarget = MathUtil.clamp(
      (0.0136 + 0.234 * DistanceToGoal + -0.0205 * Math.pow(DistanceToGoal, 2)),
      0.261799, 0.785398
    );

    s_HoodTiltMotor.set(
      (TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)) +
      (TurretHoodFeedForward.calculate(0))
    );
```

With:

```java
    /*
     * TURRET HOOD — zone-aware
     * Launch zone: auto-aim hood based on distance-to-goal polynomial.
     * Trench zone: retract hood to clear the trench (drive to minimum angle).
     * Far zone: skip hood control entirely (future: alternate aiming).
     */
    HoodEncoder = s_HoodTiltMotor.getEncoder();
    HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;

    Constants.FieldZone zone = Constants.FieldConstants.getFieldZone(
        questNav.RobotPose.getX(), m_alliance);

    switch (zone) {
      case LAUNCH:
        // Normal auto-aim: polynomial fit from physics analysis
        HoodThetaTarget = MathUtil.clamp(
          (0.0136 + 0.234 * DistanceToGoal + -0.0205 * Math.pow(DistanceToGoal, 2)),
          0.261799, 0.785398
        );
        s_HoodTiltMotor.set(
          (TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)) +
          (TurretHoodFeedForward.calculate(0))
        );
        break;

      case TRENCH:
        // Retract hood to minimum angle to clear the trench
        HoodThetaTarget = 0.261799;
        s_HoodTiltMotor.set(
          (TurretHoodPID.calculate(HoodThetaActual, HoodThetaTarget)) +
          (TurretHoodFeedForward.calculate(0))
        );
        break;

      case FAR:
        // No hood control — future: alternate aiming behavior
        break;
    }
```

- [ ] **Step 4: Guard pose-dependent logic with QuestNav tracking check**

In `periodic()`, after the test mode early return, wrap the pose-dependent block (turret pose calculation through hood control) in a tracking guard. This ensures the shooter flywheel and SmartDashboard telemetry still run even when QuestNav hasn't started tracking yet.

After:
```java
    if (m_testMode) {
      return;
    }
```

Add:
```java
    // Only run pose-dependent aiming if QuestNav has valid tracking data
    if (questNav.isTracking()) {
```

Then close the brace just before the `TURRET SHOOTER` section (before line 161). The shooter, SmartDashboard telemetry, and tuning values all remain outside the guard so they run unconditionally.

This uses the `QuestSubsystem.isTracking()` wrapper method added in Task 4.

- [ ] **Step 5: Verify it compiles**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/TurretSubsystem.java src/main/java/frc/robot/subsystems/QuestSubsystem.java
git commit -m "Add alliance-aware aiming and field zone hood logic to turret"
```

---

## Chunk 3: AutoBuilder and Auto Chooser

### Task 6: Configure AutoBuilder and Replace Auto Chooser in RobotContainer

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`

- [ ] **Step 1: Add imports**

Add these imports at the top of `RobotContainer.java`:

```java
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
```

(Some imports like `SwerveRequest` may already exist — only add the new ones.)

- [ ] **Step 2: Add ApplyRobotSpeeds request field**

After the existing `m_fieldCentricRequest` field (around line 77), add:

```java
  private final SwerveRequest.ApplyRobotSpeeds m_robotSpeedsRequest =
      new SwerveRequest.ApplyRobotSpeeds();
```

- [ ] **Step 3: Add alliance cache field**

After the auto chooser field, add:

```java
  private DriverStation.Alliance m_cachedAlliance = DriverStation.Alliance.Blue;
```

- [ ] **Step 4: Configure AutoBuilder in the constructor**

In the constructor, add AutoBuilder configuration BEFORE `configureAutoChooser()`. Replace the constructor:

```java
  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureTestBindings();
    configureAutoChooser();
  }
```

With:

```java
  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureTestBindings();
    configureAutoBuilder();
    configureAutoChooser();
  }
```

- [ ] **Step 5: Add configureAutoBuilder method**

Add this new method before `configureAutoChooser()`:

```java
  // ─── AutoBuilder ──────────────────────────────────────────────────────────

  private void configureAutoBuilder() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
        m_drive.getDrivetrain()::getPose2d,
        m_drive.getDrivetrain()::seedPose,
        m_drive.getDrivetrain()::getChassisSpeeds,
        (speeds, feedforwards) -> m_drive.getDrivetrain().setControl(
            m_robotSpeedsRequest.withSpeeds(speeds)),
        new PPHolonomicDriveController(
            new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0),  // Translation PID — TODO: tune
            new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0)   // Rotation PID — TODO: tune
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        m_drive
    );

    // Register named commands BEFORE building autos
    NamedCommands.registerCommand("shoot", new Score(m_manipulator).withTimeout(3.0));
  }
```

- [ ] **Step 6: Change m_autoChooser field declaration to non-final**

The field at line 69 currently reads:
```java
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
```

Replace with:
```java
  private SendableChooser<Command> m_autoChooser;
```

This is required because `AutoBuilder.buildAutoChooser()` returns a new instance that must be assigned to the field.

- [ ] **Step 7: Replace configureAutoChooser with automatic chooser**

Replace the entire `configureAutoChooser()` method:

```java
  private void configureAutoChooser() {
    m_autoChooser.setDefaultOption("None", Commands.none());

    m_autoChooser.addOption("CH", Commands.none());
    m_autoChooser.addOption("CH-Prime", Commands.none());
    m_autoChooser.addOption("FLS", Commands.none());
    m_autoChooser.addOption("FRS", Commands.none());
    m_autoChooser.addOption("LS", Commands.none());
    m_autoChooser.addOption("RS", Commands.none());
    m_autoChooser.addOption("Dummy", Commands.none());

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }
```

With:

```java
  private void configureAutoChooser() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }
```

- [ ] **Step 8: Add seedAutoStartPose() method**

Add this public method after `getAutonomousCommand()`:

```java
  /**
   * Seeds QuestNav with the starting pose for the selected autonomous routine.
   * Also caches the alliance color for the turret subsystem.
   * Call from Robot.autonomousInit() before scheduling the auto command.
   */
  public void seedAutoStartPose() {
    // Cache alliance
    var allianceOpt = DriverStation.getAlliance();
    m_cachedAlliance = allianceOpt.orElse(DriverStation.Alliance.Blue);
    m_TurretSubsystem.setAlliance(m_cachedAlliance);

    // Determine starting pose from selected auto name
    Command selected = m_autoChooser.getSelected();
    String autoName = selected != null ? selected.getName() : "";

    double startX, startY, startYaw;
    if (autoName.startsWith("CH")) {
      startX = AutoConstants.kCenterStartX;
      startY = AutoConstants.kCenterStartY;
      startYaw = AutoConstants.kCenterStartYaw;
    } else if (autoName.startsWith("FL")) {
      startX = AutoConstants.kFarLeftStartX;
      startY = AutoConstants.kFarLeftStartY;
      startYaw = AutoConstants.kFarLeftStartYaw;
    } else if (autoName.startsWith("FR")) {
      startX = AutoConstants.kFarRightStartX;
      startY = AutoConstants.kFarRightStartY;
      startYaw = AutoConstants.kFarRightStartYaw;
    } else if (autoName.startsWith("L")) {
      startX = AutoConstants.kLeftStartX;
      startY = AutoConstants.kLeftStartY;
      startYaw = AutoConstants.kLeftStartYaw;
    } else if (autoName.startsWith("R")) {
      startX = AutoConstants.kRightStartX;
      startY = AutoConstants.kRightStartY;
      startYaw = AutoConstants.kRightStartYaw;
    } else {
      // Default/Dummy — use center
      startX = AutoConstants.kCenterStartX;
      startY = AutoConstants.kCenterStartY;
      startYaw = AutoConstants.kCenterStartYaw;
    }

    Pose3d startPose = new Pose3d(startX, startY, 0.0,
        new Rotation3d(0.0, 0.0, startYaw));
    m_QuestSubsystem.setPose(startPose);
  }

  /**
   * Caches the alliance color and pushes it to the turret.
   * Call from teleopInit() so zone logic works in teleop too.
   */
  public void cacheAlliance() {
    var allianceOpt = DriverStation.getAlliance();
    m_cachedAlliance = allianceOpt.orElse(DriverStation.Alliance.Blue);
    m_TurretSubsystem.setAlliance(m_cachedAlliance);
  }
```

- [ ] **Step 9: Verify it compiles**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL

- [ ] **Step 10: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "Configure AutoBuilder, automatic chooser, and auto start seeding"
```

---

### Task 7: Update Robot.java to Seed Pose at Auto Start

**Files:**
- Modify: `src/main/java/frc/robot/Robot.java`

- [ ] **Step 1: Call seedAutoStartPose in autonomousInit**

Replace `autonomousInit()`:

```java
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
```

With:

```java
  @Override
  public void autonomousInit() {
    m_robotContainer.seedAutoStartPose();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
```

- [ ] **Step 2: Cache alliance in teleopInit**

In `teleopInit()`, add `cacheAlliance()` call. Replace:

```java
  @Override
  public void teleopInit() {
    // Cancel any running auto command so teleop can take over
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.configureNormalMode();
  }
```

With:

```java
  @Override
  public void teleopInit() {
    // Cancel any running auto command so teleop can take over
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.cacheAlliance();
    m_robotContainer.configureNormalMode();
  }
```

- [ ] **Step 3: Verify it compiles**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/Robot.java
git commit -m "Seed QuestNav and cache alliance at auto/teleop init"
```

---

## Chunk 4: Documentation

### Task 8: Create ABOUT4.md

**Files:**
- Create: `ABOUT4.md`

- [ ] **Step 1: Check archive directory for existing ABOUT4 files**

Run:
```bash
ls "C:/Users/hudso/clean-clones/new-repo/ABOUT-ARCHIVE/" | grep ABOUT4
```
Expected: No matches (first version will be ABOUT4a.md)

- [ ] **Step 2: Write ABOUT4.md**

Create `ABOUT4.md` in the project root with the following content:

```markdown
# Autonomous System

## Overview

The robot uses PathPlanner for trajectory generation and path following during autonomous. PathPlanner's AutoBuilder is configured with the CTRE Phoenix 6 swerve drivetrain, allowing `.auto` files to be loaded and executed as WPILib Commands directly. An automatic SendableChooser presents all available autos on the dashboard.

QuestNav is seeded with the robot's known starting pose at the beginning of each autonomous routine. The drivetrain odometry is seeded separately by PathPlanner's `resetOdom` feature.

## Path Naming Convention

Paths follow the format: `{StartPosition}-{Destination}{Segment}`

**Starting positions:**
| Prefix | Position | Approximate Coordinates (m) |
|--------|----------|----------------------------|
| FL | Far Left | (4.401, 7.625) |
| L | Left | (3.637, 5.794) |
| C | Center | (3.586, 4.067) |
| R | Right | (3.648, 2.210) |
| FR | Far Right | (4.414, 0.432) |

**Destinations:**
| Suffix | Meaning |
|--------|---------|
| S | Score — drive to a scoring position |
| H | Human Player — drive to the human player station |
| C | Curl — drive through a curling trajectory |
| D | Depot — drive to the depot |

**Segment numbers** (1, 2, etc.) indicate the order within a multi-path auto.

## Path Inventory

| Path | Start | End | Notes |
|------|-------|-----|-------|
| L-S1 | Left | Upleft Shoot (via far field) | First segment of LS auto |
| L-S2 | Upleft Shoot | Upright Shoot (via far field) | Second segment of LS auto |
| R-S1 | Right | Upleft Shoot (via far field) | First segment of RS auto |
| R-S2 | Right area | Upright Shoot (via far field) | Second segment of RS auto |
| C-H1 | Center | Human Player | First segment of CH auto |
| C-H2 | Human Player area | Return | Second segment of CH auto |
| C-D1 | Center | Depot | Standalone path |
| FL-S1 | Far Left | Upright Shoot (via far field) | First segment of FLS auto |
| FR-S1 | Far Right | Upleft Shoot (via far field) | First segment of FRS auto |
| FR-H1 | Far Right | Human Player | Standalone path |
| R-H1 | Right | Human Player | Standalone path |
| R-C1 | Right | Bumpright Shoot (curl through trench) | Standalone path |
| Dummy1 | Test path 1 | | For testing |
| Dummy2 | Test path 2 | | For testing |

## Auto Routines

| Auto | Paths | Sequence |
|------|-------|----------|
| CH | C-H1 → wait → C-H2 | Center start, drive to human player, wait (score), return |
| CH-Prime | C-H1 → wait | Truncated CH — center to human player only |
| LS | L-S1 → wait → L-S2 | Left start, score run through far field |
| RS | R-S1 → wait → R-S2 | Right start, score run through far field |
| FLS | FL-S1 → wait → L-S2 | Far left start, then joins LS second segment |
| FRS | FR-S1 → wait → R-S2 | Far right start, then joins RS second segment |
| Dummy | Test auto | For testing path following |

The 5-second waits between path segments are placeholders for scoring actions (`"shoot"` NamedCommand).

## Field Zones

The field is divided into three zones measured from the friendly alliance wall:

| Zone | Range (inches) | Range (meters) | Hood Behavior |
|------|---------------|-----------------|---------------|
| Launch | 0 – 156.61 | 0 – 3.978 | Auto-aim normally |
| Trench | 156.61 – 201.01 | 3.978 – 5.106 | Retract to clear trench |
| Far | 201.01+ | 5.106+ | No hood control (future) |

Zone logic is global — it runs in both autonomous and teleop via `TurretSubsystem.periodic()`.

## Alliance Awareness

- **Path mirroring:** PathPlanner automatically flips paths for red alliance (blue-origin coordinate system)
- **Goal targeting:** Turret aims at blue or red goal based on `DriverStation.getAlliance()`
  - Blue goal: (4.635, 4.034, 1.0)
  - Red goal: (11.9068, 4.034, 1.0) — field-mirrored from blue (X only)
- **Zone calculation:** Distance from friendly wall is computed using alliance color
```

- [ ] **Step 3: Archive ABOUT4.md**

```bash
cp ABOUT4.md "C:/Users/hudso/clean-clones/new-repo/ABOUT-ARCHIVE/ABOUT4a.md"
```

- [ ] **Step 4: Commit**

```bash
git add ABOUT4.md
git commit -m "Add ABOUT4.md documenting autonomous paths and system"
```

---

### Task 9: Update ABOUT1.md with Red Goal Note

**Files:**
- Modify: `ABOUT1.md`

- [ ] **Step 1: Add red goal mirroring note**

In the TurretSubsystem section of ABOUT1.md (around the goal position constants area), add a note after the existing turret description. Find the text about "Blue Alliance Goal Position" or the turret constants and add:

```
Note: The red alliance goal position is computed by mirroring the blue goal across the field X-axis (only X is flipped, Y is preserved). These are configurable placeholder values pending real field measurements. See Constants.java TurretSubsystemConstants for the actual values.
```

- [ ] **Step 2: Archive ABOUT1.md**

First check the archive for the latest ABOUT1 letter:
```bash
ls "C:/Users/hudso/clean-clones/new-repo/ABOUT-ARCHIVE/" | grep ABOUT1
```
Then copy with the next available letter suffix.

- [ ] **Step 3: Commit**

```bash
git add ABOUT1.md
git commit -m "Add red goal mirroring note to ABOUT1.md"
```

---

## Chunk 5: Build Verification

### Task 10: Final Build and Smoke Test

- [ ] **Step 1: Clean build**

Run:
```bash
./gradlew clean build
```
Expected: BUILD SUCCESSFUL

- [ ] **Step 2: Verify auto chooser populates (simulation)**

Run:
```bash
./gradlew simulateJava
```

In the simulation, open SmartDashboard/Shuffleboard and verify:
- "Auto Chooser" widget appears with all auto names (CH, CH-Prime, FLS, FRS, LS, RS, Dummy)
- Selecting an auto and enabling autonomous does not crash

- [ ] **Step 3: Review all changes**

Run:
```bash
git log --oneline -10
git diff HEAD~8..HEAD --stat
```

Verify all expected files were modified and no unexpected files were touched.
