# FAR_AWAY Zone — Max Hood Angle Override Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When the robot is more than 12.428 m from its friendly wall (the new FAR_AWAY zone), override the hood angle target to its upper clamp value (0.785398 rad) so passing-range shots launch with the steepest feasible trajectory.

**Architecture:** Extend the existing `FieldZone` enum in `Constants.FieldConstants` with two new zones (`FAR_TRENCH`, `FAR_AWAY`) and two new distance thresholds. Consume the zone in `TurretSubsystem.periodic()` with a single conditional that overrides `HoodThetaTarget` after the existing polynomial clamp. No new subsystem wiring — the turret already has `m_alliance` and `TurretPose`.

**Tech Stack:** Java 17, WPILib 2026, Gradle (`./gradlew build` for compile verification).

**Spec:** `docs/superpowers/specs/2026-04-18-far-away-hood-max-design.md`

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/main/java/frc/robot/Constants.java` | Modify | Extend `FieldZone` enum, add `kFarZoneEndMeters` and `kFarTrenchZoneEndMeters`, update `getFieldZone()` cascade |
| `src/main/java/frc/robot/subsystems/TurretSubsystem.java` | Modify | Add FAR_AWAY hood override inside the `questNav.isTracking()` branch of `periodic()` |

No new files. No vendordep changes. No new tests (project has no unit test infrastructure; verification is `./gradlew build` for compilation and manual sim/robot drive for behavior — same convention used by `2026-03-17-autonomous-implementation.md`).

---

## Chunk 1: Extend FieldZone Enum and Thresholds

### Task 1: Add FAR_TRENCH and FAR_AWAY to FieldZone

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java:254`

- [ ] **Step 1: Extend the `FieldZone` enum**

Find line 254 in `Constants.java`:

```java
  public enum FieldZone { LAUNCH, TRENCH, FAR }
```

Replace with:

```java
  public enum FieldZone { LAUNCH, TRENCH, FAR, FAR_TRENCH, FAR_AWAY }
```

- [ ] **Step 2: Verify compilation**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL. (Adding enum values is backward-compatible; no existing switch statements currently consume `FieldZone`, so no non-exhaustive-switch warnings.)

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/Constants.java
git commit -m "feat(field): add FAR_TRENCH and FAR_AWAY zones to FieldZone enum"
```

---

### Task 2: Add kFarZoneEndMeters and kFarTrenchZoneEndMeters Constants

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java:262-265` (inside `FieldConstants` class)

- [ ] **Step 1: Add the two new distance constants**

Find the existing trench-end constant block in `FieldConstants` (around lines 262–265):

```java
    /** End of launch zone, measured from friendly wall (156.61 inches). */
    public static final double kLaunchZoneEndMeters = 3.978;
    /** End of trench zone, measured from friendly wall (201.01 inches). */
    public static final double kTrenchZoneEndMeters = 5.106;
```

Add two more lines immediately after `kTrenchZoneEndMeters`:

```java
    /** End of launch zone, measured from friendly wall (156.61 inches). */
    public static final double kLaunchZoneEndMeters = 3.978;
    /** End of trench zone, measured from friendly wall (201.01 inches). */
    public static final double kTrenchZoneEndMeters = 5.106;
    /** End of far zone, measured from friendly wall. */
    public static final double kFarZoneEndMeters = 11.3;
    /** End of far-trench zone, measured from friendly wall. */
    public static final double kFarTrenchZoneEndMeters = 12.428;
```

- [ ] **Step 2: Verify compilation**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/Constants.java
git commit -m "feat(field): add kFarZoneEndMeters and kFarTrenchZoneEndMeters thresholds"
```

---

### Task 3: Update getFieldZone() Cascade

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java:274-289` (the `getFieldZone` method body)

- [ ] **Step 1: Replace the classification cascade**

Find the current body of `getFieldZone()` (around lines 274–289). Replace the `if/else if/else` classification block so the full method reads:

```java
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
      } else if (distFromFriendlyWall <= kFarZoneEndMeters) {
        return FieldZone.FAR;
      } else if (distFromFriendlyWall <= kFarTrenchZoneEndMeters) {
        return FieldZone.FAR_TRENCH;
      } else {
        return FieldZone.FAR_AWAY;
      }
    }
```

- [ ] **Step 2: Verify compilation**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/Constants.java
git commit -m "feat(field): classify FAR_TRENCH and FAR_AWAY in getFieldZone cascade"
```

---

## Chunk 2: TurretSubsystem Hood Override

### Task 4: Add FieldZone Imports to TurretSubsystem

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/TurretSubsystem.java:25` (import section)

- [ ] **Step 1: Import `FieldConstants` and `FieldZone`**

`Constants.java` and `TurretSubsystem.java` are in different packages (`frc.robot` vs `frc.robot.subsystems`). Find line 25 in `TurretSubsystem.java`:

```java
import frc.robot.Constants;
```

Replace with:

```java
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldZone;
```

- [ ] **Step 2: Verify compilation**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL. (Unused imports won't fail the build but will compile cleanly; they're consumed in Task 5.)

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/TurretSubsystem.java
git commit -m "chore(turret): import FieldConstants and FieldZone"
```

---

### Task 5: Override HoodThetaTarget in FAR_AWAY Zone

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/TurretSubsystem.java:288-291` (just after the polynomial hood clamp, inside the `questNav.isTracking()` branch)

- [ ] **Step 1: Add the zone override immediately after the polynomial clamp**

Find the existing hood target calculation in `periodic()` (around lines 286–291):

```java
      // Calculate  Turret Hood  * * * * *
      HoodEncoder = s_HoodTiltMotor.getEncoder();
      HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;
      HoodThetaTarget = MathUtil.clamp(
        (0.0136 + 0.234 * DistanceToGoal + -0.0205 * (DistanceToGoal * DistanceToGoal)),
        0.261799, 0.785398
      );
```

Add the override block immediately after the closing `);` of the clamp (still inside the `if (questNav.isTracking())` block, before `// Calculate Turret Shooter`):

```java
      // Calculate  Turret Hood  * * * * *
      HoodEncoder = s_HoodTiltMotor.getEncoder();
      HoodThetaActual = (((HoodEncoder.getPosition()) / (Constants.TurretSubsystemConstants.HoodGearRatio)) * (2 * Math.PI)) + 0.261799;
      HoodThetaTarget = MathUtil.clamp(
        (0.0136 + 0.234 * DistanceToGoal + -0.0205 * (DistanceToGoal * DistanceToGoal)),
        0.261799, 0.785398
      );
      if (FieldConstants.getFieldZone(TurretPose.getX(), m_alliance) == FieldZone.FAR_AWAY) {
        HoodThetaTarget = 0.785398;
      }
```

Notes for the implementer:
- Placement is **after** the clamp so the override wins regardless of what the polynomial produced.
- The override is **only** inside the `questNav.isTracking()` branch; the non-tracking fallback at lines 306–315 (with its fixed `0.558` rad hood target) is intentionally untouched.
- `TurretPose.getX()` matches the coordinate source the existing pass-aim X-threshold block (lines 219–246) already uses — not `drive.getPose2d().getX()`, not `questNav.RobotPose.getX()`.
- `m_alliance` is the existing private field set via `setAlliance()`; do not introduce any new alliance lookup.
- The literal `0.785398` is intentional and inline, matching the style of the adjacent clamp expression. Do **not** extract a `kHoodMaxAngle` constant; the spec explicitly rejects that.

- [ ] **Step 2: Verify compilation**

Run:
```bash
./gradlew build
```
Expected: BUILD SUCCESSFUL.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/TurretSubsystem.java
git commit -m "feat(turret): override hood to max angle in FAR_AWAY zone"
```

---

## Chunk 3: Verification

### Task 6: Manual Sim/Robot Verification

**Files:** None modified. Verification only.

This project has no JUnit test suite; behavior verification happens on the robot or in simulation via SmartDashboard/NetworkTables inspection. Complete **all** of the following checks before declaring the feature done.

- [ ] **Step 1: Boundary check — Blue alliance, entering FAR_AWAY**

Set up: Blue alliance selected, QuestNav tracking, drive to `X = 12.4 m`.
Expected: `HoodThetaTarget` on SmartDashboard/NT follows the polynomial (not 0.785398).

Drive to `X = 12.5 m`.
Expected: `HoodThetaTarget == 0.785398` exactly.

- [ ] **Step 2: Boundary check — Red alliance, entering FAR_AWAY**

Set up: Red alliance selected, QuestNav tracking. Red FAR_AWAY starts at `X <= 16.5418 − 12.428 = 4.1138 m`.
Drive to `X = 4.2 m`.
Expected: `HoodThetaTarget` follows the polynomial.

Drive to `X = 4.0 m`.
Expected: `HoodThetaTarget == 0.785398`.

- [ ] **Step 3: Exit check — leaving FAR_AWAY into FAR_TRENCH**

Blue alliance: drive from `X = 13.0 m` back to `X = 12.0 m`.
Expected: `HoodThetaTarget` transitions from `0.785398` back to the polynomial value.

- [ ] **Step 4: QuestNav-lost fallback sanity**

Blue alliance, at `X = 13.0 m` (inside FAR_AWAY), force `questNav.isTracking()` to return false (disconnect Quest or spoof).
Expected: `HoodThetaTarget` becomes `0.558` (the fallback value at line 311), **not** `0.785398`. The override must not apply when QuestNav is not tracking.

- [ ] **Step 5: Unchanged-zone regression**

Blue alliance, drive through `X = 2 m` (LAUNCH), `X = 4.5 m` (TRENCH), `X = 8 m` (FAR), `X = 11.8 m` (FAR_TRENCH).
Expected: `HoodThetaTarget` follows the polynomial in all four zones — no override applied.

- [ ] **Step 6: Turret rotate unchanged**

In FAR_AWAY (e.g. Blue at `X = 13 m`), verify `TurretThetaTarget` is aiming at `AimPointB1` or `AimPointB2` (depending on Y), just as it does at `X = 8 m`. The turret rotate behavior must be unchanged by this feature.

- [ ] **Step 7: No commit in this task**

Verification only — no code changes. Do not create a commit for this task.

---

## Self-Review Notes

- **Spec coverage:** every item in `Changes` section of the spec is covered: FieldZone enum extension (Task 1), two new constants (Task 2), getFieldZone cascade update (Task 3), hood override in TurretSubsystem (Tasks 4–5). All `Testing` items from the spec are in Task 6.
- **Out-of-scope adherence:** no TRENCH/FAR_TRENCH retraction logic, no pass-aim threshold change, no `kHoodMaxAngle` named constant, no changes to non-tracking fallback — all explicitly matching spec's "Out of scope" section.
- **No placeholders:** all code blocks show exact content; all commands are complete; no "TBD" or "handle edge cases".
- **Type consistency:** `FieldZone`, `FieldConstants`, `kFarZoneEndMeters`, `kFarTrenchZoneEndMeters`, `HoodThetaTarget`, `m_alliance`, `TurretPose` — names consistent across all tasks.
