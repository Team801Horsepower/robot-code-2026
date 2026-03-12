# Test Mode Design

## Overview

A hardware checkout mode using WPILib's built-in Test mode. When "Test" is selected in Driver Station, all non-drivetrain motors run at 0.1 power via dedicated button bindings, and all motor outputs/encoder readings are logged to NetworkTables.

Drivetrain motors are exempt from the 0.1 power rule and retain normal drive controls.

## Architecture

Uses the WPILib Test mode lifecycle in Robot.java:

- `testInit()` — Cancel all commands, call `RobotContainer.configureTestMode()`
- `testPeriodic()` — No-op (telemetry handled via subsystem periodic or registerTelemetry callbacks)
- `testExit()` — Cancel test commands, call `RobotContainer.configureNormalMode()`

RobotContainer gains two public methods:
- `configureTestMode()` — Registers test-mode default commands and button bindings
- `configureNormalMode()` — Restores normal default commands and bindings (also called from `teleopInit`)

Each non-drivetrain subsystem gains a `testRun(double power)` method that directly sets motor output, bypassing PID/jam detection/feedforward. Turret exposes separate methods per motor group.

## Button Mapping

Drive controls (right stick translation, left stick rotation) remain identical to teleop.

| Button | Motor | Power |
|--------|-------|-------|
| Left Trigger (held) | Gather | +0.1 |
| Left Bumper (held) | Gather | -0.1 |
| Right Trigger (held) | Feeder | +0.1 |
| D-pad Down (held) | Feeder | -0.1 |
| Right Bumper (held) | Spindex | +0.1 |
| A (held) | Launch wheels 1 & 2 | +0.1 / -0.1 (counter-rotate) |
| B (held) | Hood | +0.1 |
| D-pad Right (held) | Hood | -0.1 |
| X (held) | Turret rotate | +0.1 |
| D-pad Left (held) | Turret rotate | -0.1 |
| Y (held) | Hopper | +0.1 |
| D-pad Up (held) | Hopper | -0.1 |
| Start (held) | Level1 | +0.1 |
| Back (held) | Level1 | -0.1 |

## Telemetry

Published to `TestMode/<Subsystem>/` NetworkTables table during test mode. DataLogManager auto-captures to USB.

Per motor:
- `Power` — applied motor output
- `Position` — encoder position (where available)
- `Velocity` — encoder velocity (where available)

| Subsystem | Power | Position | Velocity |
|-----------|-------|----------|----------|
| Gather | Yes | No | No |
| Hopper | Yes | Yes (inches) | Yes |
| Spindex | Yes | No | Yes (RPM) |
| Feeder | Yes | No | No |
| Launch 1 | Yes | No | Yes (RPM) |
| Launch 2 | Yes | No | Yes (RPM) |
| Hood | Yes | Yes (rotations) | No |
| Turret Rotate | Yes | Yes (degrees) | No |
| Level1 | Yes | Yes (rotations) | No |
