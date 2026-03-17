# Score While Moving Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable the robot to score while translating by adding a drive speed multiplier during shooting and turret rotation lead compensation.

**Architecture:** When the right trigger exceeds 0.15, drive velocities are scaled down. When it exceeds 0.25, the launch sequence fires. The turret adds an angular lead offset to its rotation target based on the robot's field-relative velocity perpendicular to the goal line. Hood and flywheel models are untouched.

**Tech Stack:** WPILib 2026.2.1, CTRE Phoenix 6 swerve, REV SparkFlex, Java 17

**Spec:** `docs/superpowers/specs/2026-03-17-score-while-moving-design.md`

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `src/main/java/frc/robot/Constants.java` | Modify | Add 3 new constants |
| `src/main/java/frc/robot/subsystems/TurretSubsystem.java` | Modify | Add velocity supplier, lead compensation in periodic() |
| `src/main/java/frc/robot/RobotContainer.java` | Modify | Wire supplier, update trigger threshold, add speed multiplier |

No new files.

---

## Chunk 1: Constants and Speed Multiplier

### Task 1: Add new constants to Constants.java

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java:115-130` (DriveConstants)
- Modify: `src/main/java/frc/robot/Constants.java:149-187` (TurretSubsystemConstants)

- [ ] **Step 1: Add speed multiplier constants to DriveConstants**

In `Constants.java`, inside the `DriveConstants` class (after line 129, before the closing brace on line 130), add:

```java
    /** Drive speed multiplier applied when right trigger > kShootSlowdownThreshold. */
    public static final double kShootWhileMovingSpeedMultiplier = 0.3;

    /** Right trigger axis value at which the speed multiplier activates. */
    public static final double kShootSlowdownThreshold = 0.15;
```

- [ ] **Step 2: Add lead factor constant to TurretSubsystemConstants**

In `Constants.java`, inside the `TurretSubsystemConstants` class (after line 186, before the closing brace on line 187), add:

```java
    /** Multiplier on turret lead offset. 0 = disabled, 1 = full compensation. */
    public static final double kLeadFactor = 1.0;
```

- [ ] **Step 3: Build to verify constants compile**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/Constants.java
git commit -m "feat: add score-while-moving constants (speed multiplier, lead factor)"
```

---

### Task 2: Add speed multiplier to RobotContainer drive logic

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java:96-110` (buildFieldCentricRequest)
- Modify: `src/main/java/frc/robot/RobotContainer.java:129-131` (right trigger binding)

- [ ] **Step 1: Add speed multiplier in buildFieldCentricRequest()**

Replace the `buildFieldCentricRequest()` method (lines 96-110) with:

```java
  private SwerveRequest.FieldCentric buildFieldCentricRequest() {
    double speedScale = 1.0;
    if (m_driverController.getRightTriggerAxis() > DriveConstants.kShootSlowdownThreshold) {
      speedScale = DriveConstants.kShootWhileMovingSpeedMultiplier;
    }

    double translationX = applyDeadband(-m_driverController.getLeftY())
        * DriveConstants.kMaxSpeedMetersPerSecond * speedScale;

    double translationY = applyDeadband(-m_driverController.getLeftX())
        * DriveConstants.kMaxSpeedMetersPerSecond * speedScale;

    double rotation = applyDeadband(-m_driverController.getRightX())
        * DriveConstants.kMaxAngularSpeedRadPerSec * speedScale;

    return m_fieldCentricRequest
        .withVelocityX(translationX)
        .withVelocityY(translationY)
        .withRotationalRate(rotation);
  }
```

This reads the raw right trigger axis (0.0-1.0) and applies the multiplier to all three velocity components when it exceeds the threshold.

- [ ] **Step 2: Update right trigger shoot threshold from 0.15 to 0.25**

In `configureButtonBindings()`, change line 130:

```java
    // Old:
    m_driverController
        .rightTrigger(0.15)
        .whileTrue(new Shoot(m_launch));

    // New:
    m_driverController
        .rightTrigger(0.25)
        .whileTrue(new Shoot(m_launch));
```

- [ ] **Step 3: Build to verify**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "feat: apply drive speed multiplier when right trigger held for shooting"
```

---

## Chunk 2: Turret Lead Compensation

### Task 3: Add ChassisSpeeds supplier to TurretSubsystem

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/TurretSubsystem.java:1-94` (imports, fields, constructor)
- Modify: `src/main/java/frc/robot/RobotContainer.java:46` (TurretSubsystem construction)

- [ ] **Step 1: Add imports to TurretSubsystem.java**

Add these imports at the top of TurretSubsystem.java (after the existing imports, before the class declaration):

```java
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
```

Note: `Rotation2d` may already be available via the wildcard `edu.wpi.first.math.geometry.*` — but it's not imported that way in this file. Check if `Rotation2d` resolves; if not, add the explicit import. `ChassisSpeeds` and `Supplier` are definitely new.

- [ ] **Step 2: Add supplier field and update constructor**

In TurretSubsystem.java, add a field after the `questNav` field (after line 41):

```java
  private final Supplier<ChassisSpeeds> m_chassisSpeedsSupplier;
```

Update the constructor signature (line 82) to accept the supplier:

```java
  public TurretSubsystem(QuestSubsystem questNav, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.questNav = questNav;
    this.m_chassisSpeedsSupplier = chassisSpeedsSupplier;
```

The rest of the constructor body remains unchanged.

- [ ] **Step 3: Wire the supplier in RobotContainer.java**

In RobotContainer.java, update the TurretSubsystem construction (line 46) from:

```java
  private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(m_QuestSubsystem);
```

To:

```java
  private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(
      m_QuestSubsystem, () -> m_drivetrain.getState().Speeds);
```

`m_drivetrain.getState()` returns CTRE's `SwerveDriveState`, and `.Speeds` is a robot-relative `ChassisSpeeds`. The field-relative conversion happens inside TurretSubsystem.

- [ ] **Step 4: Build to verify**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/TurretSubsystem.java src/main/java/frc/robot/RobotContainer.java
git commit -m "feat: pass ChassisSpeeds supplier to TurretSubsystem for lead compensation"
```

---

### Task 4: Implement lead compensation in TurretSubsystem.periodic()

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/TurretSubsystem.java:119-140` (distance calc through turret rotate motor set)

- [ ] **Step 1: Move BallVelocityTarget calculation before turret rotate section**

`BallVelocityTarget` is currently computed on line 169 (in the TURRET SHOOTER section). The lead compensation needs it earlier. Move this single line up so it's computed right after `DistanceToGoal` (after line 123):

```java
    DistanceToGoal = Math.sqrt(vX*vX + vY*vY);

    // Ball velocity needed for both lead compensation and shooter control
    BallVelocityTarget = 5.58 + 0.38 * DistanceToGoal + -0.0394 * Math.pow(DistanceToGoal, 2);
```

Then in the TURRET SHOOTER section (original line 169), remove the duplicate `BallVelocityTarget = ...` line since it's now computed earlier.

- [ ] **Step 2: Add lead compensation logic**

In `periodic()`, replace lines 133-140 (the `TurretThetaTarget` calculation through `s_TurretRotateMotor.set(...)`) with:

```java
    double rawTurretTarget = ((-Math.atan2(vY, vX) + (TurretYaw)) +
        Constants.TurretSubsystemConstants.TurretRotateScoreOffset);

    // ── Lead compensation ──────────────────────────────────────────────
    // Skip lead calc if too close to goal (avoid division by zero)
    double leadOffset = 0.0;
    if (DistanceToGoal > 0.1 && BallVelocityTarget > 0.1) {
      // Convert robot-relative chassis speeds to field-relative
      // Note: TurretYaw equals robot heading because RobotToTurretYaw = 0.0
      // Note: ChassisSpeeds source is CTRE odometry; heading is from QuestNav
      ChassisSpeeds robotSpeeds = m_chassisSpeedsSupplier.get();
      ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
          robotSpeeds, Rotation2d.fromRadians(TurretYaw));

      // Unit vector from turret to goal
      double uX = vX / DistanceToGoal;
      double uY = vY / DistanceToGoal;

      // Perpendicular direction (90° CCW rotation of unit vector)
      // Signed dot product gives velocity component perpendicular to goal line
      double vPerp = fieldSpeeds.vxMetersPerSecond * (-uY)
                   + fieldSpeeds.vyMetersPerSecond * uX;

      // Lead offset: atan(v_perp / ball_velocity) — distance cancels out of flight time
      leadOffset = Math.atan(vPerp / BallVelocityTarget)
                        * Constants.TurretSubsystemConstants.kLeadFactor;
    }

    TurretThetaTarget = MathUtil.clamp(rawTurretTarget + leadOffset, -3.49066, 3.49066);

    SmartDashboard.putNumber("LeadOffset", Math.toDegrees(leadOffset));

    s_TurretRotateMotor.set(
    (TurretRotatePID.calculate(TurretThetaActual, TurretThetaTarget)) +
    (TurretRotateFeedForward.calculate(0))
    );
```

**Key details:**
- `BallVelocityTarget` is now computed before this block (Step 1)
- Guard clause: skips lead compensation if `DistanceToGoal <= 0.1` or `BallVelocityTarget <= 0.1` (avoids division by zero / NaN)
- `TurretYaw` is the robot heading (because `RobotToTurretYaw = 0.0`) — used for field-relative conversion
- `vX` and `vY` are the turret-to-goal vector components already computed above
- The lead offset is added **before** the clamp, so near turret limits it degrades gracefully
- `LeadOffset` is published in degrees for intuitive debugging in SmartDashboard/AdvantageScope

- [ ] **Step 3: Build to verify**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Verify by grepping for key terms**

Run: `grep -n "leadOffset\|kLeadFactor\|vPerp\|LeadOffset\|kShootWhileMovingSpeedMultiplier\|kShootSlowdownThreshold" src/main/java/frc/robot/subsystems/TurretSubsystem.java src/main/java/frc/robot/RobotContainer.java src/main/java/frc/robot/Constants.java`

Expected: All new terms appear in their expected files.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/TurretSubsystem.java
git commit -m "feat: add turret rotation lead compensation for score-while-moving"
```

---

## Chunk 3: Documentation and Verification

### Task 5: Update documentation files

**Files:**
- Modify: `ABOUT1.md` (spindexer/turret description)
- Modify: `ABOUT3.md` (controls mapping)

- [ ] **Step 1: Update ABOUT3.md controls**

Update the Right Trigger line to reflect the new threshold:

```
Right Trigger: Launch, where the button is pressed more than 0.25, begin the launch sequence. When pressed more than 0.15, the robot's drive speed is reduced to allow scoring while moving.
```

- [ ] **Step 2: Update ABOUT1.md turret description**

In the turret paragraph, add a sentence noting the lead compensation:

> The turret automatically compensates for robot velocity by leading its rotation target, allowing scoring while moving at reduced drive speed.

- [ ] **Step 3: Archive ABOUT files per CLAUDE.md**

List the archive directory, find next letter suffixes, copy modified files.

- [ ] **Step 4: Commit**

```bash
git add ABOUT1.md ABOUT3.md
git commit -m "docs: update controls and turret description for score-while-moving"
```

---

### Task 6: Final verification

- [ ] **Step 1: Full build**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL

- [ ] **Step 2: Review all changes**

Run: `git diff HEAD~4` (or however many commits back to before Task 1)
Verify:
- Constants.java has 3 new constants
- TurretSubsystem.java has supplier field, updated constructor, lead compensation in periodic()
- RobotContainer.java has supplier wiring, speed multiplier in buildFieldCentricRequest(), trigger threshold updated to 0.25
- No unintended changes to hood or flywheel logic

- [ ] **Step 3: On-robot lead direction test (when hardware available)**

Drive laterally past the goal while monitoring `LeadOffset` on SmartDashboard. Verify the turret leads in the correct direction. If the lead is reversed, negate `kLeadFactor` (set to `-1.0` instead of `1.0`).
