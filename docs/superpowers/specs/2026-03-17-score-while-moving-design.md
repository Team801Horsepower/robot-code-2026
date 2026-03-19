# Score While Moving — Design Spec

## Problem

The robot currently must stop to score. The turret auto-aims using static geometry (atan2 to goal, distance-based hood/flywheel models) with no velocity compensation. When moving, shots miss laterally because the turret aims at the goal's current bearing rather than leading the target.

## Requirements

- Robot can score while translating at moderate speed on relatively straight paths
- Works across the turret's full engagement range
- Always-on: no mode toggle — the system compensates automatically when shooting
- Speed multiplier (< 1) applied to drive when shooting to reduce error
- Minimal new code; keep proven hood angle and flywheel velocity quadratic models unchanged
- Configurable lead factor (can be zeroed out to disable lead compensation)

## Design

### 1. Speed Multiplier on Drive

When the right trigger exceeds a slowdown threshold, the swerve drive velocities are scaled by a configurable multiplier.

**Behavior:**
- Right trigger > 0.15 (`kShootSlowdownThreshold`): translation AND rotation velocities multiplied by `kShootWhileMovingSpeedMultiplier` (default 0.3). Scaling rotation reduces heading change during the shot.
- Right trigger > 0.25: launch sequence fires (existing Shoot command binding, threshold updated from 0.15)
- This creates a ~0.10 buffer zone where the robot slows before the launch sequence activates

**Location:** `RobotContainer.buildFieldCentricRequest()` — read raw right trigger axis via `m_driverController.getRightTriggerAxis()` (returns 0.0-1.0) and scale the translation/rotation values before applying to the swerve request.

### 2. Turret Rotation Lead Compensation

A small angular offset is added to the turret rotation target to compensate for lateral robot motion.

**Algorithm (in TurretSubsystem.periodic(), after existing TurretThetaTarget calculation):**

1. Get robot-relative `ChassisSpeeds` from a `Supplier<ChassisSpeeds>` (provided via constructor). The source is `DrivetrainSubsystem.getState().Speeds` (CTRE `SwerveDriveState.Speeds`), which is robot-relative.
2. Convert to field-relative speeds using robot heading:
   ```java
   ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
       robotSpeeds, Rotation2d.fromRadians(heading));
   ```
3. Compute unit vector from turret to goal: `u = (goalPos - turretPos) / distance` (turret-to-goal vector already computed)
4. Compute signed perpendicular velocity via dot product with the 90-degree CCW rotation of the unit vector:
   ```
   perp_direction = (-u_y, u_x)
   v_perp = dot(fieldVel, perp_direction)
   ```
   The sign of `v_perp` indicates which side of the goal line the robot is moving toward.
5. Compute angular offset. Since `flightTime = distance / ballVelocity`, the distance cancels:
   ```
   offset = atan(v_perp / ballVelocity) * kLeadFactor
   ```
6. Add offset to `TurretThetaTarget` **before** the existing clamp to `[-3.49066, 3.49066]`. Near turret limits, lead compensation degrades gracefully as the clamp absorbs the offset.

**Coupling:** TurretSubsystem receives `Supplier<ChassisSpeeds>` in its constructor, wired in RobotContainer as `() -> m_drivetrain.getState().Speeds`. This keeps TurretSubsystem decoupled from DrivetrainSubsystem.

**Simplifications acknowledged:**
- Ball flight time uses muzzle velocity, not average velocity (ball decelerates due to drag). Acceptable as a first approximation at reduced robot speeds.
- Pure rotational velocity is not compensated by this algorithm. The turret PID's tracking bandwidth handles rotational rates since rotation changes the bearing directly and the PID responds every loop.
- Alliance goal coordinates are hardcoded to blue (pre-existing; lead compensation inherits this).

### 3. Constants

**New constants in `DriveConstants`:**
- `kShootWhileMovingSpeedMultiplier = 0.3` — drive speed cap while shooting
- `kShootSlowdownThreshold = 0.15` — right trigger value where multiplier activates

**New constant in `TurretSubsystemConstants`:**
- `kLeadFactor = 1.0` — multiplier on lead offset (0 = disabled, 1 = full compensation)

**Updated constant:**
- Shoot trigger threshold in `configureButtonBindings()`: 0.15 -> 0.25

## Files Changed

| File | Change |
|------|--------|
| `Constants.java` | Add `kShootWhileMovingSpeedMultiplier`, `kShootSlowdownThreshold`, `kLeadFactor` |
| `TurretSubsystem.java` | Add `Supplier<ChassisSpeeds>` constructor param + field; ~10 lines in `periodic()` for lead calc; publish lead offset to NetworkTables for tuning |
| `RobotContainer.java` | Wire supplier to TurretSubsystem; update shoot trigger threshold to 0.25; add multiplier logic in `buildFieldCentricRequest()` |

No new files. No new classes. Hood angle and flywheel velocity models are untouched.

**Note:** Test mode (`m_testMode = true`) causes `periodic()` to skip auto-aim, so lead compensation is automatically disabled in test mode.

## Tuning Notes

- `kShootWhileMovingSpeedMultiplier`: Lower = more accurate but slower driving. Start at 0.3, increase if shots land reliably.
- `kLeadFactor`: Start at 1.0. If overshooting laterally, reduce toward 0. If undershooting, values slightly above 1.0 are fine.
- If lead compensation causes issues, setting `kLeadFactor = 0` gracefully falls back to multiplier-only mode.
- Monitor the `LeadOffset` telemetry value on SmartDashboard to verify the compensation direction and magnitude during testing.
