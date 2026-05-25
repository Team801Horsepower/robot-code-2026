Turret Aiming Math:



## Overview

The turret subsystem (`TurretSubsystem.java`) solves three problems every 20 ms cycle:

1. **Where to rotate** the turret (yaw) to face the goal
2. **What hood angle** to set for the ball to reach the goal
3. **How fast to spin** the flywheel so the ball arrives with the correct velocity

All calculations depend on the robot's field-relative pose from QuestNav (Meta Quest 6DOF tracking) and the known position of the scoring goal.



## 1\. Coordinate Systems and Pose Pipeline

**QuestNav** (`QuestSubsystem.java`) tracks the Meta Quest headset pose and publishes:

* `QuestPose` — the raw 3D headset pose in field coordinates
* `RobotPose` — headset pose transformed by `QuestToRobot` (X: -0.263 m, Y: 0.229 m, Yaw: π)

`QuestSubsystem` also feeds vision measurements into the drivetrain's pose estimator each cycle via `m_drivetrain.addVisionMeasurement()`.

The **turret pose** is derived directly from the Quest headset pose using a separate `QuestToTurret` transform:

```
TurretPose = QuestPose * QuestToTurret
```

Where `QuestToTurret` has offsets:

* X: -0.016 m
* Y: 0.323 m
* Z: 0 m
* Yaw: π (180°, turret is mounted facing opposite the headset)

The robot's **heading** for yaw calculations comes from the drivetrain's pose estimator (`drive.getPose2d().getRotation()`), not from QuestNav directly.

> **Note:** `Constants.TurretSubsystemConstants` also defines `RobotToTurret` offsets (X: -0.103165 m, Y: -0.094050 m, Z: 0.5588 m) — these are stored for reference but the actual turret pose computation uses `QuestToTurret` above.



## 2\. Goal Selection (Alliance-Aware Aim Points)

There are 6 aim points (3 per alliance). The turret selects which one to target based on the turret's X and Y position.

**Blue Alliance:**

|Aim Point|Field Coordinates (X, Y, Z)|When Selected|
|-|-|-|
|AimPointB1|(1.978, 5.759, 1.8288)|X > 4.626, Y > 4.035|
|AimPointB2|(1.978, 2.310, 1.8288)|X > 4.626, Y < 4.035|
|BlueAllianceGoal|(4.635, 4.034, 1.8288)|X <= 4.626 (near goal)|

**Red Alliance:**

|Aim Point|Field Coordinates (X, Y, Z)|When Selected|
|-|-|-|
|AimPointR1|(14.563, 2.310, 1.8288)|X < 11.915, Y < 4.035|
|AimPointR2|(14.563, 5.759, 1.8288)|X < 11.915, Y > 4.035|
|RedAllianceGoal|(11.907, 4.034, 1.8288)|X >= 11.915 (near goal)|

All aim points are at Z = 1.8288 m (72 inches), the height of the scoring target. Exact selection thresholds from code: X = 4.625594 (blue), X = 11.915394 (red), Y = 4.034536.

The static vector from turret to goal (before velocity adjustment):

```
staticGoalX = GoalX - TurretPose.getX()
staticGoalY = GoalY - TurretPose.getY()
```



## 3\. Velocity Estimation and Time-of-Flight Lead

**Robot velocity** is estimated by finite difference of `questNav.RobotPose` between cycles, smoothed through a 5-sample moving average filter:

```
TurretVx = movingAverage((RobotPose.X - RobotPose.X_prev) / 0.02)
TurretVy = movingAverage((RobotPose.Y - RobotPose.Y_prev) / 0.02)
```

**Time of flight** is a polynomial fit against static distance to goal:

```
D_static = sqrt(staticGoalX^2 + staticGoalY^2)
TimeOfFlight = 0.976 + 0.0027 * D_static + 0.00677 * D_static^2
```

The lead-compensated aim vectors are computed by projecting where the goal will be relative to the robot after the ball is in flight:

```
vX = (GoalX - TurretVx * TimeOfFlight) - TurretPose.getX()
vY = (GoalY - TurretVy * TimeOfFlight) - TurretPose.getY()
```

**Horizontal distance** used for all subsequent polynomial evaluations:

```
D = sqrt(vX^2 + vY^2)
```



## 4\. Turret Rotation (Yaw Aiming)

### 4.1 Base Target Angle

The turret target yaw is the bearing from the turret to the lead-compensated goal, adjusted for the robot's heading from the drivetrain odometry:

```
TurretYaw = -atan2(vY, vX) + RobotRotation
```

Where `RobotRotation` is `drive.getPose2d().getRotation().getRadians()`.

> **Note:** `TurretRotateScoreOffset = -0.797 rad` is defined in `Constants.TurretSubsystemConstants` but is not applied in the current `periodic()` implementation.

### 4.2 Angle Wrapping and Clamping

The turret has a physical range of +/-3.49066 rad (+/-200 deg):

1. Wrap `TurretYaw` into \[-π, π]
2. Check three candidate angles: `TurretYaw`, `TurretYaw + 2*π`, `TurretYaw - 2*π`
3. Select the candidate closest to the current turret position that falls strictly within (-3.49066, 3.49066)
4. Clamp result to \[-3.49066, 3.49066]

### 4.3 PID Control

```
Motor output = PID(TurretThetaActual, TurretThetaTarget) + Feedforward(0)
```

* PID: P = 1.5, I = 0, D = 0
* Feedforward gains are all 0 (placeholder)
* Encoder: absolute DutyCycleEncoder on DIO port 0, offset by `TurretRotateOffset = 3.66519 rad`



## 5\. Flywheel Velocity

### 5.1 Ball Velocity Polynomial

The required ball exit velocity (m/s) as a function of horizontal distance (meters):

```
V_ball = 6 - 0.00447 * D + 0.104 * D^2
```

This is a quadratic fit from physics analysis. At close range (~2 m), V_ball is approximately 6.4 m/s. At longer range (~6 m), V_ball is approximately 9.7 m/s.

### 5.2 Motor RPM Conversion

```
ShooterVelocityTarget = (60 * V_ball) / WheelCircumference
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



## 6\. Hood Angle

### 6.1 Flat-Ground Polynomial

The hood tilt angle (radians) as a function of the lead-compensated horizontal distance D:

```
theta_hood = 0.0136 + 0.234 * D - 0.0205 * D^2
```

This polynomial was **empirically tuned on flat ground** with the turret at 22 inches (0.5588 m) and the goal at 72 inches (1.8288 m), giving a baseline vertical displacement of:

```
BaselineDeltaZ = 1.8288 - 0.5588 = 1.27 m
```

The hood angle is clamped to \[0.261799, 0.785398] rad, which is \[15 deg, 45 deg].

### 6.2 FAR_AWAY Zone Override

When the robot is in the `FAR_AWAY` field zone (determined by `FieldConstants.getFieldZone()`), the hood is overridden to maximum elevation regardless of distance:

```
if zone == FAR_AWAY:
    HoodThetaTarget = 0.785398  (45 deg, maximum)
```

### 6.3 Hood Encoder

The hood uses a relative encoder (NEO Vortex internal) with a gear ratio of 26.25:1.

```
theta_actual = (encoder_position / 26.25) * 2*pi + 0.261799
```

The `+ 0.261799` offset means encoder zero corresponds to the minimum hood angle (15 deg).

### 6.4 Hood PID Control

```
Motor output = PID(HoodThetaActual, HoodThetaTarget) + Feedforward(0)
```

* PID: P = 0.7, I = 0, D = 0.0085
* Feedforward gains are all 0



## 7\. Hood Angle — Vertical Displacement Compensation (Design/Future)

> **Note:** The vertical displacement compensation described in this section is **not implemented** in the current `TurretSubsystem.java`. The flat-ground polynomial (Section 6) is used directly with no pitch, roll, or Z adjustments. This section documents the intended design for tilt-compensated shooting.

### 7.1 The Problem

The flat-ground polynomial does not account for two effects when the robot is on a tilted surface:

1. **Height change**: The turret's actual Z position changes, altering the vertical distance to the goal
2. **Frame coupling**: Nonzero pitch/roll causes turret yaw rotation to also affect field-frame launch elevation

### 7.2 Elevation Correction

```
elevationCorrection = atan2(vZ, D) - atan2(BaselineDeltaZ, D)
```

On flat ground, `vZ = 1.27 m = BaselineDeltaZ`, so correction is exactly 0.

### 7.3 Full 3D Hood Computation

```
theta_field = polynomial(D) + elevationCorrection
aim_yaw = atan2(vY, vX)

field_dir = (cos(theta_field) * cos(aim_yaw),
             cos(theta_field) * sin(aim_yaw),
             sin(theta_field))

robot_dir = Rotation3d_inverse(TurretRotation) * field_dir
horiz = sqrt(robot_dir.x^2 + robot_dir.y^2)
theta_hood = atan2(robot_dir.z, horiz)
```

Clamped to \[15 deg, 45 deg]. When the robot is flat, this reduces identically to the polynomial output.



## 8\. Fallback Mode (No QuestNav Tracking)

When QuestNav does not have valid tracking data, the turret uses safe defaults:

* `ShooterVelocityTarget` = 1754.46 RPM (pre-spin for immediate readiness)
* `HoodThetaTarget` = 0.558 rad (~32 deg, mid-range elevation)
* `TurretThetaTarget` = -1.5708 rad (-π/2, turret faces right in robot frame)

Motor PID loops still run — they drive toward these default targets.



## 9\. Control Flow Summary

```
periodic() [every 20 ms]:
    Read encoders: TurretThetaActual, HoodThetaActual, ShooterVelocityActual

    Compute TurretPose = QuestPose * QuestToTurret
    RobotRotation = drive.getPose2d().getRotation()
    Estimate TurretVx, TurretVy via filtered finite difference of RobotPose
    Compute TimeOfFlight polynomial (static distance)
    Compute DistanceToGoal from lead-adjusted vX, vY

    if testMode -> return (manual control only)

    if QuestNav.isTracking() and turretAutoAimEnabled:
        1. Select aim point (GoalX, GoalY) based on TurretPose and alliance
        2. Compute lead-adjusted vX, vY (goal position minus velocity * time-of-flight)
        3. Compute TurretYaw = -atan2(vY, vX) + RobotRotation
        4. Wrap/select/clamp TurretThetaTarget
        5. Compute HoodThetaTarget = clamp(polynomial(D), 15deg, 45deg)
        6. If FAR_AWAY zone: HoodThetaTarget = 45 deg (override)
        7. Compute BallVelocityTarget (polynomial), ShooterVelocityTarget (RPM)
    else:
        ShooterVelocityTarget = 1754.46 RPM
        HoodThetaTarget = 0.558 rad
        TurretThetaTarget = -1.5708 rad

    Drive turret rotate motor (PID)
    Drive shooter motors (PID + FF)
    Store RobotPose as previous for next cycle's velocity estimate

    if hoodAutoAimEnabled:
        Drive hood motor to HoodThetaTarget (PID)
    else:
        Drive hood motor to 15 deg (retracted)
```



## 10\. SmartDashboard Telemetry

As of current code, SmartDashboard publishing in `TurretSubsystem` is not active (section marked `// NA`). The turret pose is published to NetworkTables via a `StructPublisher<Pose2d>` on the `"TurretPose"` topic. `QuestSubsystem` publishes robot pose to `"MyPose"`.
