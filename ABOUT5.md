Turret Aiming Math:



## Overview

The turret subsystem (`TurretSubsystem.java`) solves three problems every 20 ms cycle:

1. **Where to rotate** the turret (yaw) to face the goal
2. **What hood angle** to set for the ball to reach the goal
3. **How fast to spin** the flywheel so the ball arrives with the correct velocity

All calculations depend on the robot's field-relative pose from QuestNav (Meta Quest 6DOF tracking) and the known position of the scoring goal.



## 1\. Coordinate Systems and Pose Pipeline

**QuestNav** provides a full `Pose3d` (X, Y, Z, roll, pitch, yaw) of the robot in field coordinates.

The turret is offset from the Quest headset mounting point. A rigid-body `Transform3d` converts the robot pose to the turret pose:

```
TurretPose = RobotPose \* RobotToTurret
```

Where `RobotToTurret` has offsets:

* X: -0.103165 m (turret is behind Quest mount)
* Y: -0.094050 m (turret is right of Quest mount)
* Z: 0.5588 m (turret launch point is 22 inches above ground)
* Roll, Pitch, Yaw: all 0 (turret base is aligned with robot frame)

From the turret pose we extract:

* `TurretX`, `TurretY`, `TurretZ` — field-frame position
* `TurretYaw` — robot heading (equals `TurretRotation.getZ()`)
* `TurretRotation` — full 3D orientation (used for tilt compensation)



## 2\. Goal Selection (Alliance-Aware Aim Points)

There are 6 aim points (3 per alliance). The turret selects which one to target based on the robot's zone on the field.

**Blue Alliance:**

|Aim Point|Field Coordinates (X, Y, Z)|When Selected|
|-|-|-|
|AimPointB1|(3.978, 5.759, 1.829)|X > 4.626, Y > 4.035|
|AimPointB2|(3.978, 2.310, 1.829)|X > 4.626, Y < 4.035|
|BlueAllianceGoal|(4.635, 4.034, 1.829)|X <= 4.626 (near goal)|

**Red Alliance:**

|Aim Point|Field Coordinates (X, Y, Z)|When Selected|
|-|-|-|
|AimPointR1|(12.563, 2.310, 1.829)|X < 11.915, Y < 4.035|
|AimPointR2|(12.563, 5.759, 1.829)|X < 11.915, Y > 4.035|
|RedAllianceGoal|(11.907, 4.034, 1.829)|X >= 11.915 (near goal)|

All aim points are at Z = 1.8288 m (72 inches), the height of the scoring target.

The vector from turret to goal:

```
vX = GoalX - TurretX
vY = GoalY - TurretY
vZ = GoalZ - TurretZ
```



## 3\. Distance Calculation

**Horizontal distance** (used as input to the polynomial fits):

```
D = sqrt(vX^2 + vY^2)
```

This is intentionally 2D. The polynomial equations for hood angle and ball velocity were empirically tuned against horizontal distance on flat ground. Vertical displacement is handled separately (see Section 7).



## 4\. Turret Rotation (Yaw Aiming)

### 4.1 Base Target Angle

The turret aims at the goal by computing the bearing from the turret to the goal, adjusted for the robot's heading and a mechanical scoring offset:

```
theta\_raw = -atan2(vY, vX) + TurretYaw + TurretRotateScoreOffset
```

Where `TurretRotateScoreOffset = -0.797 rad` accounts for the mechanical difference between encoder zero and the actual forward-scoring direction.

### 4.2 Lead Compensation

When the robot is moving, the ball has the robot's velocity added to the launch velocity. To compensate, the turret leads the target by an angle proportional to the robot's velocity perpendicular to the goal line.

1. Convert chassis speeds from robot-relative to field-relative:

```
   fieldSpeeds = fromRobotRelativeSpeeds(robotSpeeds, TurretYaw)
   ```

2. Compute the unit vector from turret to goal:

```
   uX = vX / D
   uY = vY / D
   ```

3. Compute the robot's velocity perpendicular to the goal line (signed dot product with 90 deg CCW rotation of unit vector):

```
   v\_perp = vx\_field \* (-uY) + vy\_field \* uX
   ```

4. Lead offset:

```
   leadOffset = atan(v\_perp / V\_ball) \* kLeadFactor
   ```

   Where `kLeadFactor = 2.0`. Note that distance cancels out of the flight time calculation, making lead compensation range-independent.

5. The lead offset is added to `theta\_raw`.

   ### 4.3 Radial Velocity Compensation

   When the robot moves toward or away from the goal (radial motion), the ball inherits that velocity component. This affects three things: flight time, required muzzle velocity, and effective distance.

   **Radial velocity** is the dot product of the field-relative velocity with the unit vector toward the goal:

   ```
   v\_radial = vx\_field \* uX + vy\_field \* uY
   ```

   Positive = approaching the goal; negative = retreating.

   **1. Corrected lead formula:** The flight time denominator uses the effective ball-to-goal speed instead of raw ball velocity:

   ```
   effectiveBallSpeed = V\_ball + v\_radial \* kRadialVelocityFactor
   leadOffset = atan(v\_perp / effectiveBallSpeed) \* kLeadFactor
   ```

   When approaching, flight time is shorter, so less lateral lead is needed. When retreating, more lead is needed.

   **2. Flywheel velocity adjustment:** The ball's effective speed toward the goal includes the robot's radial velocity. To maintain the correct arrival velocity, reduce the muzzle velocity when approaching and increase it when retreating:

   ```
   adjustedBallVelocity = V\_ball - v\_radial \* kRadialVelocityFactor
   ShooterVelocityTarget = (60 \* adjustedBallVelocity) / WheelCircumference
   ```

   At max shooting speed (1.5 m/s), this is a 15–25% adjustment on ball velocities of 6–10 m/s.

   **3. Effective distance for hood angle:** The hood polynomial uses the predicted distance the ball actually travels, not the static turret-to-goal distance:

   ```
   flightTime = D / effectiveBallSpeed
   D\_effective = D - v\_radial \* kRadialVelocityFactor \* flightTime
   ```

   The hood polynomial (Section 6) uses `D_effective` instead of `D`.

   **Safety properties:**
   * When stationary, `v_radial = 0` and all corrections are zero — identical to static behavior
   * `adjustedBallVelocity` is clamped to a minimum of 1.0 m/s
   * `effectiveDistance` is clamped to a minimum of 0.5 m
   * `kRadialVelocityFactor = 0` disables all radial corrections independently from `kLeadFactor`
   * `kRadialVelocityFactor` default = 1.0 (full physics-based compensation)

   ### 4.4 Angle Wrapping and Clamping

   The turret has a physical range of +/-3.49066 rad (+/-200 deg). After adding the lead offset:

1. Wrap `theta\_raw` into \[-pi, pi]
2. Check three candidate angles: `theta\_raw`, `theta\_raw + 2\*pi`, `theta\_raw - 2\*pi`
3. Select the candidate closest to the current turret position that is within the physical limits
4. Clamp to \[-3.49066, 3.49066]

   ### 4.5 PID Control

   ```
Motor output = PID(TurretThetaActual, TurretThetaTarget) + Feedforward(0)
```

* PID: P = 1.5, I = 0, D = 0
* Feedforward gains are all 0 (placeholder)
* Encoder: absolute DutyCycleEncoder on DIO, offset by `TurretRotateOffset = 3.66519 rad`



  ## 5\. Flywheel Velocity

  ### 5.1 Ball Velocity Polynomial

  The required ball exit velocity (m/s) as a function of horizontal distance (meters):

  ```
V\_ball = 6 - 0.00447 \* D + 0.104 \* D^2
```

  This is a quadratic fit from physics analysis. At close range (\~2m), V\_ball is approximately 6.4 m/s. At longer range (\~6m), V\_ball is approximately 9.7 m/s.

  ### 5.2 Motor RPM Conversion

  ```
ShooterVelocityTarget = (60 \* V\_ball) / WheelCircumference
```

  Where `WheelCircumference = 0.2394 m`. This converts ball velocity (m/s) to flywheel RPM.

  ### 5.3 Dual Flywheel Control

  The shooter uses two flywheels (left and right) spinning in opposite directions:

* Right motor: `+ShooterVelocityPIDSet` voltage
* Left motor: `-ShooterVelocityPIDSet` voltage (inverted)

  Velocity feedback comes from the right flywheel encoder.

  ### 5.4 PID + Feedforward

  ```
ShooterVelocityPIDSet = PID(actual, target) + Feedforward(target)
```

* PID: P = 0.0021, I = 0.00085, D = 0.000085
* I-Zone = 50 RPM (integral only accumulates when error < 50 RPM)
* Feedforward: kS = 0, kV = 0.00195, kA = 0



  ## 6\. Hood Angle — Flat-Ground Polynomial

  The hood tilt angle (radians) as a function of horizontal distance (meters). When the robot is moving, `D_effective` from Section 4.3 is used instead of `D`:

  ```
theta\_hood = 0.0136 + 0.234 \* D\_effective - 0.0205 \* D\_effective^2
```

  This polynomial was **empirically tuned on flat ground** with the turret at 22 inches (0.5588 m) and the goal at 72 inches (1.8288 m), giving a baseline vertical displacement of:

  ```
BaselineDeltaZ = 1.8288 - 0.5588 = 1.27 m
```

  The hood angle is clamped to \[0.261799, 0.785398] rad, which is \[15 deg, 45 deg].

  ### 6.1 Hood Encoder

  The hood uses a relative encoder (NEO Vortex internal) with a gear ratio of 26.25:1.

  ```
theta\_actual = (encoder\_position / 26.25) \* 2\*pi + 0.261799
```

  The `+ 0.261799` offset means encoder zero corresponds to the minimum hood angle (15 deg).

  ### 6.2 Hood PID Control

  ```
Motor output = PID(HoodThetaActual, HoodThetaTarget) + Feedforward(0)
```

* PID: P = 0.9, I = 0.003, D = 0
* Feedforward gains are all 0



  ## 7\. Hood Angle — Vertical Displacement Compensation

  > **Note:** All corrections in this section (elevation correction, pitch/roll inverse rotation, and Z-height compensation) are **only active when the `Climb` command is running**. When not climbing, `elevationCorrection = 0` and `requiredHoodAngle = polynomial(D)` — the flat-ground polynomial is used directly with no pitch, roll, yaw, or Z adjustments. The `Climb` command is currently a boilerplate command that is never bound to a button or auto routine, so these corrections are effectively disabled.

  ### 7.1 The Problem

  The flat-ground polynomial (Section 6) does not account for two effects that arise when the robot is on a tilted surface (ramp, bump, uneven field):

1. **Height change**: The turret's actual Z position changes, altering the vertical distance to the goal (vZ differs from the 1.27 m baseline)
2. **Frame coupling**: When the robot has nonzero pitch and/or roll, the turret's rotation axis (robot Z) is no longer vertical. This means turret yaw rotation also changes the field-frame launch elevation — the hood no longer has independent control over vertical aim.

   ### 7.2 Step 1: Elevation Correction

   The polynomial was tuned for a fixed baseline height difference. When the actual height difference changes, the required launch elevation changes:

   ```
elevationCorrection = atan2(vZ, D) - atan2(BaselineDeltaZ, D)
```

   On flat ground, `vZ = GoalZ - TurretZ = 1.8288 - 0.5588 = 1.27 = BaselineDeltaZ`, so the correction is exactly 0.

   ### 7.3 Step 2: Desired Field-Frame Elevation

   The polynomial output is a robot-frame hood angle that, on flat ground, equals the desired field-frame launch elevation (since robot frame = field frame when flat). Adding the elevation correction gives the desired field-frame elevation at the current height:

   ```
theta\_field = polynomial(D) + elevationCorrection
```

   ### 7.4 Step 3: Construct Field-Frame Launch Direction

   Build a unit vector pointing in the desired direction (field-frame yaw toward goal, field-frame elevation from Step 2):

   ```
aim\_yaw = atan2(vY, vX)

field\_dir = (
    cos(theta\_field) \* cos(aim\_yaw),
    cos(theta\_field) \* sin(aim\_yaw),
    sin(theta\_field)
)
```

   ### 7.5 Step 4: Inverse Rotation to Robot Frame

   Transform the desired field-frame direction into the robot's frame using the **inverse** of the robot's full 3D orientation:

   ```
robot\_dir = Rotation3d\_inverse(field\_dir)
```

   This step is where the pitch/roll/yaw coupling is resolved. The full `Rotation3d` (quaternion-based in WPILib) correctly handles:

* Pure pitch (robot tilted forward/backward)
* Pure roll (robot tilted left/right)
* Combined pitch + roll + yaw (all three changing simultaneously)
* The fact that turret yaw rotation (around tilted robot Z) also affects field-frame elevation

  In code, WPILib provides `Translation3d.rotateBy(Rotation3d)` and `Rotation3d.unaryMinus()` for the inverse.

  ### 7.6 Step 5: Extract Hood Angle

  The required robot-frame hood angle is the elevation of the robot-frame direction vector:

  ```
horiz = sqrt(robot\_dir.x^2 + robot\_dir.y^2)
theta\_hood = atan2(robot\_dir.z, horiz)
```

  Clamped to \[15 deg, 45 deg] as before.

  ### 7.7 Flat-Ground Safety Property

  When the robot is on flat ground:

* `TurretRotation` is approximately the identity rotation (pitch and roll near zero)
* The inverse rotation is also approximately identity
* `robot\_dir` approximately equals `field\_dir`
* `theta\_hood` approximately equals `theta\_field` approximately equals `polynomial(D)`

  The compensation has **zero effect on flat ground**. The proven polynomial operates unmodified.



  ## 8\. Fallback Mode (No QuestNav Tracking)

  When QuestNav does not have valid tracking data, the turret uses safe defaults:

* `ShooterVelocityTarget` = 1754.46 RPM (pre-spin for immediate readiness)
* `HoodThetaTarget` = 0.785398 rad (45 deg, maximum elevation)
* `TurretThetaTarget` = 0 (face forward)

  Motor PID loops still run — they drive toward these default targets.



  ## 9\. Control Flow Summary

  ```
periodic() \[every 20 ms]:
    if testMode -> return (manual control only)

    if QuestNav.isTracking():
        1. Transform robot pose to turret pose
        2. Select aim point based on zone and alliance
        3. Compute vX, vY, vZ, DistanceToGoal (2D)
        4. Compute BallVelocityTarget (polynomial)
        5. Compute ShooterVelocityTarget (RPM conversion, default)
        6. Compute vRadial, vPerp from field-relative chassis speeds
        7. Adjust ShooterVelocityTarget for radial velocity
        8. Compute effectiveDistance for hood polynomial
        9. Compute TurretThetaTarget (bearing + lead with corrected flight time + wrap)
        10. Compute HoodThetaTarget (polynomial on effectiveDistance; vertical displacement compensation only if climbing)
    else:
        Use fallback defaults

    Drive turret rotate motor (PID)
    Drive shooter motors (PID + FF)

    if hoodAutoAimEnabled:
        Drive hood motor to HoodThetaTarget (PID)
    else:
        Drive hood motor to 15 deg (retracted)
```



  ## 10\. SmartDashboard Telemetry

  Key values published for tuning and debugging:

|Key|Unit|Description|
|-|-|-|
|DistanceToGoal|m|2D horizontal distance to selected aim point|
|RadialVelocity|m/s|Robot velocity toward (+) or away from (-) goal|
|AdjustedBallVelocity|m/s|Flywheel target after radial velocity correction|
|EffectiveDistance|m|Predicted ball travel distance accounting for radial motion|
|LeadOffset|deg|Turret lead compensation angle|
|TurretEncoderActual|rad|Current turret yaw from encoder|
|TurretPositionTarget|rad|Target turret yaw|
|ShooterVelocityTarget|RPM|Desired flywheel speed|
|ShooterVelocityActual|RPM|Measured flywheel speed|
|Hood/Polynomial|deg|Raw polynomial output (flat-ground baseline)|
|Hood/ElevationCorrection|deg|Height difference correction|
|Hood/RequiredHoodAngle|deg|Final hood angle after all corrections|
|Hood/TurretZ|m|Turret height in field frame|
|Hood/RobotPitch|deg|Robot pitch from QuestNav|
|Hood/RobotRoll|deg|Robot roll from QuestNav|



