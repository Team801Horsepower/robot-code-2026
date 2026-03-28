# README Rewrite Design

## Problem

The current README references Python tooling (`pyproject.toml`) that doesn't exist in this Java/Gradle project. It is outdated, inaccurate, and missing key information about the tech stack, project documentation, and setup instructions.

## Goal

Rewrite the README so it is correct, clear, and useful to both team members deploying code and outside FRC community members browsing the repo.

## Approach

Self-contained overview (Approach B): enough context to understand the project without opening other files, while referencing the ABOUT files for deeper detail rather than duplicating them.

## Sections

### 1. Header & Intro

- Title: `robot-code-2026`
- Team 801 Horsepower, 2026 FRC season
- One-liner: Java/WPILib codebase for a swerve-drive robot with an auto-aiming turret and PathPlanner autonomous routines
- Note that it is an active competition codebase

### 2. Tech Stack

Concise list of key technologies with versions where relevant:

- WPILib 2026.2.1 / GradleRIO / Java 17
- CTRE Phoenix 6 (swerve drivetrain, Pigeon 2.0 IMU)
- REV Robotics (NEO Vortex motors via SparkFlex)
- PathPlanner (autonomous path following)
- AdvantageKit (logging/telemetry)
- QuestNav (Meta Quest-based pose estimation)

No deep explanations — just enough for someone to know what libraries they'll encounter.

### 3. Project Documentation (ABOUT Files)

Short paragraph explaining that detailed specs live in numbered ABOUT files, followed by:

- **ABOUT1.md** — Robot hardware specs, software versions, subsystem methods, and command reference
- **ABOUT2.md** — CAN bus ID map for all motors, encoders, and sensors
- **ABOUT3.md** — Controller mapping and operator controls
- **ABOUT4.md** — Autonomous system: PathPlanner setup, field zones, path naming, available routines

One-line descriptions only. No duplicating ABOUT file content.

### 4. Setup & Deployment

1. **Prerequisites** — Install WPILib 2026 (bundles VS Code and JDK 17)
2. **Clone** — `git clone` command
3. **Open** — Open the project folder in WPILib VS Code
4. **Deploy** — Preferred: WPILib VS Code deploy button (W logo > Deploy Robot Code). Alternative: `./gradlew deploy` from terminal.

### 5. Contributors

Listed as primary developers, no roles or bios:

- Hudson Jimenez
- Robert Ward
- Omkar Subramaniam

## Removed Content

- Python/pyproject.toml references (incorrect for this Java project)
- QuestNav Contract section (tangential; QuestNav is mentioned in the tech stack instead)
