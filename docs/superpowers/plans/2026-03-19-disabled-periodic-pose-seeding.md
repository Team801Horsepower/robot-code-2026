# Disabled-Periodic Pose Seeding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Seed both QuestNav and drivetrain odometry from the start-position chooser continuously during disabled mode, so AdvantageScope shows the correct pose before auto is enabled.

**Architecture:** Move the pose-seeding logic out of `autonomousInit()` into `disabledPeriodic()`, so the drivetrain and QuestNav update live as the operator changes the dropdown. Remove the hardcoded RobotStart1Test seeds from the `RobotContainer` constructor and `QuestSubsystem` constructor. Keep `seedAutoStartPose()` for auto init but have it call the same shared seeding logic. Cache the last-seeded pose to avoid hammering QuestNav with redundant `setPose` calls every 20ms.

**Tech Stack:** WPILib (Java), CTRE Phoenix 6 swerve, QuestNav

---

## File Structure

| File | Action | Responsibility |
|------|--------|---------------|
| `src/main/java/frc/robot/Robot.java` | Modify | Add `disabledPeriodic()` call to seed poses from chooser |
| `src/main/java/frc/robot/RobotContainer.java` | Modify | Add `seedFromChooser()` method with change detection, remove hardcoded constructor seed, update `seedAutoStartPose()` |
| `src/main/java/frc/robot/subsystems/QuestSubsystem.java` | Modify | Remove hardcoded constructor seed and dead `RobotStartingPosition` field |

---

### Task 1: Remove hardcoded seed and dead field from QuestSubsystem constructor

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/QuestSubsystem.java:24-31` (dead field), `45-47` (constructor)

- [ ] **Step 1: Remove the `RobotStartingPosition` field (lines 24-31) and the `setPose` call from the constructor (line 46)**

The `RobotStartingPosition` field becomes dead code since seeding is now handled externally.

```java
// REMOVE lines 24-31 entirely:
  Pose3d RobotStartingPosition = new Pose3d(
    Constants.QuestSubsystemConstants.RobotStart1TestX,
    Constants.QuestSubsystemConstants.RobotStart1TestY,
    Constants.QuestSubsystemConstants.RobotStart1TestZ,
    new Rotation3d(
      Constants.QuestSubsystemConstants.RobotStart1TestRoll,
      Constants.QuestSubsystemConstants.RobotStart1TestPitch,
      Constants.QuestSubsystemConstants.RobotStart1TestYaw));

// Constructor BEFORE:
public QuestSubsystem() {
    setPose(RobotStartingPosition);
}

// Constructor AFTER:
public QuestSubsystem() {}
```

- [ ] **Step 2: Verify build compiles**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/QuestSubsystem.java
git commit -m "refactor: remove hardcoded pose seed and dead field from QuestSubsystem"
```

---

### Task 2: Add `seedFromChooser()` to RobotContainer and remove constructor seed

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java:101-112` (constructor)
- Modify: `src/main/java/frc/robot/RobotContainer.java:338-352` (seedAutoStartPose)
- Add field: `m_lastSeededPose` near line 82

- [ ] **Step 1: Add a cached pose field**

Add next to the `m_startPositionChooser` field (around line 82):

```java
private Pose2d m_lastSeededPose;
```

- [ ] **Step 2: Remove the hardcoded `m_drive.seedPose()` from the constructor**

```java
// BEFORE (lines 101-112):
public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureTestBindings();
    configureAutoBuilder();
    configureAutoChooser();
    m_drive.seedPose(new Pose2d(
      Constants.QuestSubsystemConstants.RobotStart1TestX,
      Constants.QuestSubsystemConstants.RobotStart1TestY,
      new Rotation2d(Constants.QuestSubsystemConstants.RobotStart1TestYaw)
    ));
}

// AFTER:
public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureTestBindings();
    configureAutoBuilder();
    configureAutoChooser();
}
```

- [ ] **Step 3: Add `seedFromChooser()` method**

Add this new public method to RobotContainer (in the Autonomous section, before `seedAutoStartPose()`):

```java
/**
 * Seeds QuestNav and drivetrain odometry from the start-position chooser.
 * Only re-seeds when the selected pose changes, to avoid hammering
 * QuestNav with redundant setPose calls every 20ms.
 */
public void seedFromChooser() {
    Pose2d selected = m_startPositionChooser.getSelected();
    if (selected == null) {
        selected = new Pose2d();
    }

    // Only re-seed when the selection actually changes
    if (selected.equals(m_lastSeededPose)) {
        return;
    }
    m_lastSeededPose = selected;

    m_QuestSubsystem.setPose(new Pose3d(selected));
    m_drive.seedPose(selected);
}
```

- [ ] **Step 4: Simplify `seedAutoStartPose()` to delegate to `seedFromChooser()`**

```java
// BEFORE:
public void seedAutoStartPose() {
    var allianceOpt = DriverStation.getAlliance();
    m_cachedAlliance = allianceOpt.orElse(DriverStation.Alliance.Blue);
    m_TurretSubsystem.setAlliance(m_cachedAlliance);

    Pose2d startPose2d = m_startPositionChooser.getSelected();
    if (startPose2d == null) {
        startPose2d = new Pose2d();
    }

    m_QuestSubsystem.setPose(new Pose3d(startPose2d));
    m_drive.seedPose(startPose2d);
}

// AFTER:
public void seedAutoStartPose() {
    var allianceOpt = DriverStation.getAlliance();
    m_cachedAlliance = allianceOpt.orElse(DriverStation.Alliance.Blue);
    m_TurretSubsystem.setAlliance(m_cachedAlliance);

    // Force re-seed even if chooser hasn't changed (auto init is authoritative)
    m_lastSeededPose = null;
    seedFromChooser();
}
```

- [ ] **Step 5: Verify build compiles**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "feat: add seedFromChooser() with change detection, remove hardcoded seed"
```

---

### Task 3: Call `seedFromChooser()` from `disabledPeriodic()`

**Files:**
- Modify: `src/main/java/frc/robot/Robot.java:56-57`

- [ ] **Step 1: Add the call in `disabledPeriodic()`**

```java
// BEFORE (lines 56-57):
@Override
public void disabledPeriodic() {}

// AFTER:
@Override
public void disabledPeriodic() {
    m_robotContainer.seedFromChooser();
}
```

- [ ] **Step 2: Verify build compiles**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/Robot.java
git commit -m "feat: seed pose from chooser during disabled mode"
```

---

## Cleanup Note

After this plan is complete, the `RobotStart1Test*` constants in `Constants.java` (lines 162-167) are no longer referenced anywhere. They can be removed in a follow-up cleanup if desired.

---

## Verification Checklist

After all tasks are complete:

1. `./gradlew build` passes
2. Deploy to robot, open AdvantageScope
3. With robot disabled, change the "Start Position" dropdown — the displayed pose should update immediately to match
4. Enable auto — pose should remain at the chooser-selected position (not jump to RobotStart1Test)
5. Verify turret aims correctly from the seeded position
