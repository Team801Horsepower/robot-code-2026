# FAR_AWAY Zone — Max Hood Angle Override

**Date:** 2026-04-18
**Status:** Approved, pending implementation plan

## Goal

When the robot is on the opposing alliance's territory beyond the far trench, override the hood angle target to its maximum clamp value so passing-range shots launch with the steepest feasible trajectory. Turret rotate, flywheel velocity, and goal-selection logic are unchanged — pass-setpoint aiming is already active at that distance due to existing X-threshold logic in `TurretSubsystem.periodic()`.

## Background

### Existing field zones

`Constants.FieldConstants` already defines a `FieldZone` enum (`LAUNCH`, `TRENCH`, `FAR`) and a `getFieldZone(robotX, alliance)` helper that returns the zone based on alliance-adjusted distance from the friendly wall. However, this enum is **not currently consumed anywhere** in the codebase — `TurretSubsystem` does its own alliance-aware X-threshold check at lines 219–246 to pick between alliance goal coordinates and pass aim points.

### Existing hood target computation

In `TurretSubsystem.periodic()` (lines 288–291), when QuestNav is tracking, the hood angle target is computed by a polynomial clamped between `0.261799` rad (~15°, minimum) and `0.785398` rad (~45°, maximum):

```java
HoodThetaTarget = MathUtil.clamp(
  (0.0136 + 0.234 * DistanceToGoal + -0.0205 * (DistanceToGoal * DistanceToGoal)),
  0.261799, 0.785398
);
```

When QuestNav is **not** tracking (lines 306–315), hood target falls back to a hard-coded `0.558` rad.

### Existing pass-setpoint aim

`TurretSubsystem.periodic()` already selects `AimPointB1`/`B2` or `AimPointR1`/`R2` (pass setpoints) whenever the robot's X is past the mid-field threshold (~4.626 m from friendly wall, expressed as `TurretPose.getX() > 4.625594` for Blue, `< 11.915394` for Red). Any robot position in the new FAR_AWAY zone is already well past this threshold, so pass-setpoint aiming requires **no change**.

## Changes

### 1. `Constants.FieldConstants` — extend zone enum and thresholds

Two additional zones map to two new distance bands measured from the friendly wall:

| Zone         | Range (m from friendly wall) | Behavior                                    |
|--------------|------------------------------|---------------------------------------------|
| LAUNCH       | 0 – 3.978                    | Unchanged                                   |
| TRENCH       | 3.978 – 5.106                | Unchanged (zone value not consumed today)   |
| FAR          | 5.106 – 11.3                 | Unchanged                                   |
| FAR_TRENCH   | 11.3 – 12.428                | Same as TRENCH (zone value not consumed)    |
| FAR_AWAY     | 12.428+                      | Hood override to maximum angle              |

Add two constants:

```java
public static final double kFarZoneEndMeters = 11.3;
public static final double kFarTrenchZoneEndMeters = 12.428;
```

Extend the enum:

```java
public enum FieldZone { LAUNCH, TRENCH, FAR, FAR_TRENCH, FAR_AWAY }
```

Update `getFieldZone()` cascade so alliance-adjusted `distFromFriendlyWall` is classified as:

```
<= kLaunchZoneEndMeters   (3.978)   → LAUNCH
<= kTrenchZoneEndMeters   (5.106)   → TRENCH
<= kFarZoneEndMeters      (11.3)    → FAR
<= kFarTrenchZoneEndMeters (12.428) → FAR_TRENCH
else                                → FAR_AWAY
```

### 2. `TurretSubsystem.periodic()` — add FAR_AWAY hood override

Immediately after the existing polynomial clamp that computes `HoodThetaTarget` (current lines 288–291), add:

```java
if (FieldConstants.getFieldZone(TurretPose.getX(), m_alliance) == FieldZone.FAR_AWAY) {
  HoodThetaTarget = 0.785398; // max hood angle
}
```

- Uses `TurretPose.getX()` so the coordinate source matches the existing pass-aim X-threshold block at lines 219–246.
- Uses the existing `m_alliance` field; no new wiring.
- Placement **after** the clamp ensures the override wins without conditional nesting.
- Only runs inside the `questNav.isTracking()` branch — the non-tracking fallback (lines 306–315) keeps its current fixed `0.558` rad target.
- The magic literal `0.785398` is intentionally inline to match the style of the adjacent clamp expression, which also hard-codes it. No new `TurretSubsystemConstants` entry.

## Data flow

```
TurretPose.getX() ──┐
                    ├──► FieldConstants.getFieldZone() ──► FieldZone
m_alliance ─────────┘                                         │
                                                              ▼
                                                 if FAR_AWAY: HoodThetaTarget = 0.785398
                                                              │
                                                              ▼
                                               existing HoodAim() / HoodReset() PID
```

## Out of scope

- Implementing TRENCH / FAR_TRENCH hood retraction behavior (documented in ABOUT4.md but not currently in code). These zones inherit today's no-override behavior.
- Changing the pass-setpoint X-threshold block (lines 219–246) to use `FieldZone` instead of its own X check. Stable code; no requirement forces touching it.
- Adding `kHoodMaxAngle` as a named `TurretSubsystemConstants` entry. Follows adjacent style of inline literal.
- Any change to the QuestNav-not-tracking fallback branch.

## Testing

Position-based override with no timing or concurrency concerns. Verify by:

1. **Boundary check — Blue alliance:** Place robot at `X = 12.4 m` (should not override, target = polynomial result) and `X = 12.5 m` (should override, target = 0.785398). Watch `HoodThetaTarget` via SmartDashboard or physical hood angle.
2. **Boundary check — Red alliance:** Place robot at `X = 16.5418 − 12.428 = 4.1138 m`. Positions with `X < 4.1138` should override; `X > 4.1138` should not.
3. **Exit check:** Drive from FAR_AWAY back into FAR_TRENCH; verify `HoodThetaTarget` returns to polynomial value.
4. **QuestNav-lost sanity:** Disable QuestNav tracking in FAR_AWAY; confirm the `0.558` fallback takes over and the override does not interfere.

## Success criteria

- `getFieldZone()` returns the correct zone for all five bands on both alliances.
- `HoodThetaTarget == 0.785398` whenever the robot is in FAR_AWAY and QuestNav is tracking.
- No behavior change for LAUNCH, TRENCH, FAR, FAR_TRENCH, or when QuestNav is not tracking.
