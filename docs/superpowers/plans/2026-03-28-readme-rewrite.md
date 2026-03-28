# README Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the README to accurately describe this Java/WPILib FRC robot codebase, replacing the outdated Python-referencing content.

**Architecture:** Single-file rewrite of `README.md` with five sections: intro, tech stack, ABOUT file guide, setup/deploy instructions, and contributors. References ABOUT files for detail rather than duplicating them.

**Tech Stack:** Markdown

**Spec:** `docs/superpowers/specs/2026-03-28-readme-rewrite-design.md`

---

### Task 1: Rewrite README.md

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Replace README.md contents**

Write the complete new README:

```markdown
# robot-code-2026

This is the robot code for FRC Team 801 Horsepower's 2026 season. It is a Java/WPILib codebase for a swerve-drive robot featuring an auto-aiming turret and PathPlanner autonomous routines. This is an active competition codebase.

## Tech Stack

- **WPILib 2026.2.1** / GradleRIO / Java 17
- **CTRE Phoenix 6** — Swerve drivetrain (Kraken X60/X44), Pigeon 2.0 IMU, CANcoders
- **REV Robotics** — NEO Vortex motors via SparkFlex
- **PathPlanner** — Autonomous path following
- **AdvantageKit** — Logging and telemetry
- **QuestNav** — Meta Quest-based pose estimation

## Project Documentation

Detailed robot specifications are organized across numbered ABOUT files in the project root:

- **[ABOUT1.md](ABOUT1.md)** — Robot hardware overview, subsystem API reference, and command definitions
- **[ABOUT2.md](ABOUT2.md)** — CAN bus ID map for all motors, encoders, and sensors
- **[ABOUT3.md](ABOUT3.md)** — Controller mapping and operator controls
- **[ABOUT4.md](ABOUT4.md)** — Autonomous system: PathPlanner setup, field zones, path naming, and available routines

## Setup & Deployment

### Prerequisites

Install [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html), which bundles VS Code and JDK 17.

### Clone

```bash
git clone https://github.com/Team801Horsepower/robot-code-2026.git
```

### Open

Open the project folder in WPILib VS Code.

### Deploy

**Preferred:** Use the WPILib button in the top-right corner of VS Code (the **W** logo) and select **Deploy Robot Code**.

**Alternative:** Run from the terminal:

```bash
./gradlew deploy
```

## Contributors

- Hudson Jimenez
- Robert Ward
- Omkar Subramaniam
```

- [ ] **Step 2: Verify the README renders correctly**

Read the file back and visually confirm all sections, links, and formatting are correct.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "Rewrite README for correctness and clarity"
```
