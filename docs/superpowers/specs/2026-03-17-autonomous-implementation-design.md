# Autonomous Implementation Design

**Date:** 2026-03-17
**Team:** 801 Horsepower
**Status:** Approved

## Prerequisites

**Install PathPlanner vendordep.** The `pathplannerlib` vendordep is not currently installed. It must be added to the `vendordeps/` directory before any implementation begins. Install via WPILib's "Manage Vendor Libraries" > "Install new library (online)" with the PathPlanner 2026 JSON URL, or download the JSON manually. Without this, none of the AutoBuilder code will compile.

## Overview

The robot has 14 PathPlanner paths and 7 auto routines (CH, CH-Prime, FLS, FRS, LS, RS, Dummy) that are fully designed in PathPlanner but not yet wired into the robot code. The auto chooser currently maps every option to `Commands.none()`.

This design covers:
1. Wiring PathPlanner autos to the CTRE swerve drivetrain via AutoBuilder
2. Adding alliance-aware field zone logic that controls hood behavior globally (auto + teleop)
3. Switching turret aiming between blue and red goals based on FMS alliance data
4. Seeding QuestNav and drivetrain odometry at autonomous start
5. Documentation (ABOUT4.md describing paths, minor ABOUT1.md correction)

## 1. AutoBuilder Configuration

### Setup

Configure `AutoBuilder` in `RobotContainer`'s constructor, before the auto chooser is built. AutoBuilder needs:

- **Get current pose:** `m_drive.getDrivetrain().getPose2d()` (via `DrivetrainSubsystem`)
- **Reset pose:** `m_drive.getDrivetrain().seedPose()` (via `DrivetrainSubsystem`)
- **Get robot-relative chassis speeds:** Add a new `getChassisSpeeds()` method to `DrivetrainSubsystem` that wraps `getState().Speeds` or uses the kinematics to convert module states
- **Drive with chassis speeds:** Create a `SwerveRequest.ApplyRobotSpeeds` instance in `RobotContainer` and apply it to `m_drive.getDrivetrain()` via `setControl()`
- **Path-following controller:** `PPHolonomicDriveController` with tuned PID gains
- **Robot config:** from PathPlanner `settings.json` (mass, MOI, module positions, etc.)
- **Alliance flipping:** `DriverStation.getAlliance()` for automatic blue-to-red path mirroring

### Auto Chooser

Replace the manual `SendableChooser` with `AutoBuilder.buildAutoChooser()`. This automatically discovers all `.auto` files in `src/main/deploy/pathplanner/autos/` and presents them in the dashboard. A "None" default is included automatically.

### NamedCommands

Register named commands before building autos:
- `"shoot"` -> `new Score(m_manipulator).withTimeout(3.0)` — the `Score` command never self-terminates (`isFinished()` returns false), so a timeout is required to prevent it from stalling the auto sequence. Tune the timeout to match actual scoring duration.

The 5-second waits in the `.auto` files should be replaced with named command references. This requires editing the `.auto` JSON files or re-doing it in the PathPlanner GUI. If waits are kept temporarily, scoring still works via the named commands.

## 2. Field Zones and Hood Management

### Zone Definitions

Measured from the friendly alliance wall:

| Zone | Start (inches) | End (inches) | Start (meters) | End (meters) | Hood Behavior |
|------|---------------|-------------|-----------------|---------------|---------------|
| Launch | 0 | 156.61 | 0 | 3.978 | Auto-aim normally |
| Trench | 156.61 | 201.01 | 3.978 | 5.106 | Retract (clear trench) |
| Far | 201.01 | field end | 5.106 | ~16.54 | No hood control (future: alternate aiming) |

### Alliance-Aware Distance Calculation

- **Blue alliance:** distance from friendly wall = robot X position (blue wall at x = 0)
- **Red alliance:** distance from friendly wall = (16.5418 - robot X position) (red wall at x = 16.5418m)

### Implementation Location

Zone logic lives in `TurretSubsystem.periodic()`, which already runs every loop in both auto and teleop.

At the top of `periodic()`:
1. Compute distance from friendly wall using QuestNav pose + cached alliance color
2. Determine zone via `getFieldZone()` utility method
3. If **Launch zone** -> run existing auto-aim hood logic
4. If **Trench zone** -> override hood to retract (drive to flat/retracted position)
5. If **Far zone** -> skip hood motor control entirely

### Data Model

```java
public enum FieldZone { LAUNCH, TRENCH, FAR }
```

Constants in a new `FieldConstants` inner class in `Constants.java`:
- `kLaunchZoneEndMeters = 3.978`
- `kTrenchZoneEndMeters = 5.106`
- `kFieldLengthMeters = 16.5418`

A static utility method in `Constants.FieldConstants`:
```java
public static FieldZone getFieldZone(double robotX, Alliance alliance)
```

This structure is extensible for future far-zone behavior.

## 3. Alliance-Aware Turret Aiming

### Current State

`TurretSubsystem.periodic()` hardcodes `BlueGoalX/Y` for aiming vector calculation.

### Changes

- Query `DriverStation.getAlliance()` and cache the result (alliance doesn't change mid-match)
- Select blue or red goal coordinates based on cached alliance
- Existing vector math stays the same: `vX = GoalX - TurretX`, etc.

### Goal Coordinates

Blue goal (existing): `(4.635, 4.034, 1.0)`

Red goal (field-mirrored, configurable placeholder):
- `RedGoalX = 16.5418 - 4.635 = 11.9068`
- `RedGoalY = 4.034` (Y is preserved in standard FRC field mirroring — alliance stations are on opposite X ends, so only X is mirrored)
- `RedGoalZ = 1.0`

Standard FRC field mirroring flips X across the field centerline while keeping Y the same. The field is 16.5418m long. These are configurable placeholders — tune with real field measurements. If the game's goals are diagonally opposite rather than directly across, Y would also need mirroring.

## 4. QuestNav Integration in Autonomous

### At Auto Start (`autonomousInit`)

1. Determine the selected auto's starting pose (from the auto's first path start waypoint, or from a map of auto-name to known starting pose)
2. Call `QuestSubsystem.setPose()` to seed QuestNav with the starting pose — this method must be added to `QuestSubsystem` as: `public void setPose(Pose3d pose) { questNav.setPose(pose); }`
3. Drivetrain odometry seeding is handled by PathPlanner's `resetOdom: true` in the `.auto` files, which calls AutoBuilder's resetPose callback automatically when the auto command starts. No manual `Drive.seedPose()` call is needed for the drivetrain — only QuestNav needs the explicit seed.

**Note on initialization order:** The QuestNav seed must happen before the auto command is scheduled, so QuestNav has the correct pose from the first loop. Guard the zone logic in `TurretSubsystem.periodic()` with a `questNav.isTracking()` check to handle the brief period before QuestNav has valid data.

### During Auto

- QuestNav updates `RobotPose` every loop via `QuestSubsystem.periodic()` (unchanged)
- Turret reads QuestNav pose for aiming (unchanged)
- Zone logic reads QuestNav pose for zone determination (new)
- PathPlanner path following uses CTRE's internal drivetrain odometry (separate from QuestNav)

### Starting Position Constants

Define named starting positions corresponding to the path linked names:
- `Center`: (3.586, 4.067)
- `Right`: (3.648, 2.210)
- `Left`: (3.637, 5.794)
- `Far Right`: (4.414, 0.432)
- `Far Left`: (4.401, 7.625)

These are extracted from the first waypoint anchors of the respective paths.

## 5. Documentation

### ABOUT4.md (New)

Describes the autonomous system:
- Path naming convention: `{StartPosition}-{Destination}{Segment}` where start positions are FL/L/C/R/FR and destinations are S(Score)/H(Human Player)/C(Curl)/D(Depot)
- Table of all 14 paths with start/end positions
- Table of all 7 autos with path sequences and actions between segments
- Field zone definitions and measurements
- Alliance awareness explanation

### ABOUT1.md (Minor Edit)

Add a note in the turret section clarifying that the red goal coordinates are field-mirrored from the blue goal values (configurable placeholders pending real measurement).

## Files Modified

| File | Change |
|------|--------|
| `RobotContainer.java` | AutoBuilder config, automatic chooser, NamedCommands registration |
| `Constants.java` | FieldConstants (zones, field length), updated RedGoal values, starting pose constants |
| `TurretSubsystem.java` | Alliance-aware aiming, field zone hood logic |
| `QuestSubsystem.java` | Add public `setPose(Pose3d)` method wrapping `questNav.setPose()` |
| `DrivetrainSubsystem.java` | Add `getChassisSpeeds()` method for AutoBuilder |
| `Robot.java` | Seed QuestNav pose in `autonomousInit()` (drivetrain seeded by PathPlanner's `resetOdom`) |
| `ABOUT4.md` | New file describing autonomous paths |
| `ABOUT1.md` | Minor note about red goal mirroring |

## Dependencies

- PathPlanner library (`pathplannerlib`) must be installed as a vendordep (see Prerequisites section)
- CTRE Phoenix 6 swerve API compatibility with PathPlanner's `AutoBuilder`

## Out of Scope

- Fusing QuestNav and drivetrain odometry (future enhancement)
- Far-zone turret behavior (future: alternate aiming target)
- Tuning path-following PID gains (separate tuning session)
- Editing `.auto` files to replace waits with NamedCommands (can be done in PathPlanner GUI)
