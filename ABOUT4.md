# Autonomous System

## Overview

The autonomous system uses PathPlanner for path following on a CTRE Phoenix 6 swerve drivetrain. Paths are pre-built in PathPlanner GUI and stored as `.auto` and `.path` files under `src/main/deploy/pathplanner/`. At runtime, `AutoBuilder.buildAutoChooser()` discovers all `.auto` files and presents them in a SmartDashboard dropdown for selection.

QuestNav (Meta Quest headset) provides robot pose data. At the start of autonomous, `RobotContainer.seedAutoStartPose()` seeds QuestNav with the known starting pose selected from a manual SmartDashboard pose chooser (`m_startPositionChooser`), not based on the auto name prefix.

Alliance color is read from FMS via `DriverStation.getAlliance()` and cached in both `autonomousInit()` and `teleopInit()`. PathPlanner uses this to automatically mirror paths for the red alliance. The turret subsystem uses it to select the correct goal coordinates and compute field zones.

## Field Zones

The field is divided into three zones measured from the friendly alliance wall:

| Zone | Distance from Friendly Wall | Hood Behavior |
|------|----------------------------|---------------|
| **Launch** | 0 – 156.61 in (0 – 3.978 m) | Auto-aiming enabled |
| **Trench** | 156.61 – 201.01 in (3.978 – 5.106 m) | Hood retracted (clearance for trench) |
| **Far** | 201.01 in+ (5.106 m+) | No hood control (future: alternate aiming) |

Zone logic is alliance-aware: for red alliance, distance is measured from the red end of the field (`fieldLength - robotX`). This zone system is active during both autonomous and teleop.

## Path Naming Convention

Paths follow the pattern `{StartPosition}-{Destination}{Segment}`:

**Starting Positions:**
- **FL** — Far Left (4.401, 7.625, 0 rad)
- **L** — Left (3.637, 5.794, PI/2 rad)
- **C** or **CH** — Center (3.586, 4.067, PI rad)
- **R** — Right (3.648, 2.210, 0 rad)
- **FR** — Far Right (4.414, 0.432, 0 rad)

**Destinations:**
- **S** — Score
- **H** — Human Player station
- **D** — Depot
- **C** — Curl

**Segments:** A number suffix (1, 2, etc.) indicates sequential segments of a multi-part path.

**Prime** suffix denotes a truncated version of the original auto.

## Available Autonomous Routines (.auto files)

| Auto | Start Position | Description |
|------|---------------|-------------|
| **CH.auto** | Center | Center start, routes toward Human Player side |
| **CH-Prime.auto** | Center | Truncated version of CH |
| **FLC.auto** | Far Left | Far Left start, routes to Curl |
| **FLS.auto** | Far Left | Far Left start, routes to Score |
| **FRC.auto** | Far Right | Far Right start, routes to Curl |
| **FRH.auto** | Far Right | Far Right start, routes to Human Player |
| **FRS.auto** | Far Right | Far Right start, routes to Score |
| **LS.auto** | Left | Left start, routes to Score |
| **RH.auto** | Right | Right start, routes to Human Player |
| **RS.auto** | Right | Right start, routes to Score |
| **Dummy.auto** | Center (default) | Placeholder/test auto |

## Available Paths (.path files)

| Path | Description |
|------|-------------|
| C-D1 | Center to Depot, segment 1 |
| C-H1, C-H2 | Center to Human Player, segments 1-2 |
| FL-C1, FL-C2 | Far Left to Curl, segments 1-2 |
| FL-S1 | Far Left to Score, segment 1 |
| FR-C1, FR-C2 | Far Right to Curl, segments 1-2 |
| FR-H1 | Far Right to Human Player, segment 1 |
| FR-S1 | Far Right to Score, segment 1 |
| L-S1, L-S2 | Left to Score, segments 1-2 |
| R-C1 | Right to Curl, segment 1 |
| R-H1 | Right to Human Player, segment 1 |
| R-S1, R-S2 | Right to Score, segments 1-2 |
| Dummy1, Dummy2 | Placeholder/test paths |

## Named Commands

PathPlanner auto sequences can trigger robot actions via named commands:

| Name | Command | Timeout |
|------|---------|---------|
| `shoot` | Sequence: enable hood auto-aim → `Score(m_manipulator, m_gather, m_hopper, m_spindex, m_feeder)` → disable hood auto-aim | None (runs until interrupted) |

## Technical Details

- **AutoBuilder** is configured in `RobotContainer.configureAutoBuilder()` with `PPHolonomicDriveController` (PID 5.0/0.0/0.0 for both translation and rotation — TODO: tune)
- **Robot config** is loaded from PathPlanner GUI settings via `RobotConfig.fromGUISettings()`
- **Drive request** uses `SwerveRequest.ApplyRobotSpeeds` for robot-relative velocity control during path following
- **QuestNav seeding** uses a SmartDashboard start position chooser (`m_startPositionChooser`) to select the starting pose, configured in `RobotContainer.configureAutoChooser()`
- **Alliance caching** happens in `seedAutoStartPose()` (auto) and `cacheAlliance()` (teleop), pushed to `TurretSubsystem.setAlliance()`
