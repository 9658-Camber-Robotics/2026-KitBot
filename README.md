# 2026-KitBot — Team 9658

Robot code for Team 9658's 2026 competition robot (FIRST Robotics Competition, REBUILT season). Built on WPILib Java with a swerve drivetrain, vision-assisted auto-aim, and a distance-adaptive shooting system.

## Table of Contents

- [Overview](#overview)
- [Tech Stack](#tech-stack)
- [Project Structure](#project-structure)
- [Robot Hardware](#robot-hardware)
- [Controls](#controls)
- [Cool Features](#cool-features)
  - [Auto-Aim](#auto-aim)
  - [Auto-Shoot / Distance-Adaptive Shot Table](#auto-shoot--distance-adaptive-shot-table)
  - [Vision Pose Estimation](#vision-pose-estimation)
  - [Alliance-Aware Field Geometry](#alliance-aware-field-geometry)
  - [Autonomous](#autonomous)
- [Building and Deploying](#building-and-deploying)
- [Tuning](#tuning)

## Overview

The robot is a swerve-drive shooter robot with three main mechanisms:

- **Drivetrain** — a swerve drive (via YAGSL) that handles both driver-relative and field-relative driving, path following, and a vision-assisted auto-aim mode.
- **Shooter** — a dual-Kraken X60 flywheel that spins up to a target RPM and can automatically choose that RPM based on distance to the target.
- **Indexer** — a NEO-driven belt/roller stage that feeds game pieces into the shooter once it's at speed.

A climber subsystem exists in the codebase but is currently disabled (commented out) while it's being developed/tuned.

## Tech Stack

| Library | Purpose |
|---|---|
| [WPILib](https://docs.wpilib.org/) | Core robot framework (command-based) |
| [YAGSL](https://github.com/BroncBotz3481/YAGSL) | Swerve drive implementation, config-driven via JSON in `src/main/deploy/swerve` |
| [YAMS](https://github.com/BroncBotz3481/YAMS) ("Yet Another Mechanism System") | Motor/mechanism abstraction (flywheels, arms) with built-in simulation and telemetry |
| [PathPlannerLib](https://pathplanner.dev/) | Autonomous path following and the auto chooser |
| [CTRE Phoenix 6](https://v6.docs.ctr-electronics.com/) | TalonFX (Kraken X60) motor control for the shooter |
| [REVLib](https://docs.revrobotics.com/) | SparkMax (NEO) motor control for the indexer |
| [LimelightLib](https://limelightvision.io/) | AprilTag-based vision pose estimation |
| [ReduxLib](https://reduxrobotics.com/) / [ThriftyLib](https://thethriftybot.com/) / [Studica](https://studica.com/) | Sensor/misc hardware vendor support |

## Project Structure

```
src/main/java/frc/robot/
├── Main.java                     Entry point (do not edit)
├── Robot.java                    TimedRobot lifecycle hooks, runs the CommandScheduler
├── RobotContainer.java           Subsystem wiring, controller bindings, autonomous/named commands
├── Constants.java                All tunable numbers: CAN IDs, motor configs, setpoints
├── commands/
│   ├── AutoAimCommand.java       Vision-assisted "point at the hub while driving" command
│   ├── AutoShoot.java            Spin up + timed feed shooting routine
│   ├── ShootAndIndexCommand.java Main shoot command: distance-based RPM lookup + feed
│   ├── IntakeCommand.java        Runs shooter+indexer in reverse-ish to intake game pieces
│   └── OuttakeCommand.java       Ejects game pieces
├── subsystems/
│   ├── SwerveSubsystem.java      Drivetrain, odometry, Limelight fusion, PathPlanner setup
│   ├── ShooterSubsystem.java     Flywheel velocity/duty-cycle control (YAMS FlyWheel)
│   ├── IndexerSubsystem.java     Feeder wheel velocity/duty-cycle control (YAMS FlyWheel)
│   └── ClimbSubsystem.java       Climber arm (currently disabled/commented out)
└── utils/
    ├── AllianceFlipUtil.java     Mirrors field poses across the red/blue alliance line
    └── FieldConstants.java       Field dimensions and landmark poses (hub, trenches, towers, etc.), derived from the AprilTag field layout
```

## Robot Hardware

CAN IDs and motor configuration live in `Constants.java`:

| Mechanism | Motor(s) | Controller | CAN ID(s) |
|---|---|---|---|
| Shooter | 2x Kraken X60 (leader + follower) | TalonFX | 4 (leader), 41 (follower, inverted) |
| Indexer | 1x NEO | SparkMax | 3 |
| Climber (disabled) | 2x Kraken X60 | TalonFX | 9, 0 |

Motor gearing, current limits, PID gains, and feedforward constants are all defined per-mechanism as `SmartMotorControllerConfig` objects in `Constants.java` — this is the first place to look when tuning.

## Controls

**Driver controller (port 0):**

| Input | Action |
|---|---|
| Left stick | Translate (field-relative) |
| Right stick | Heading/rotation |
| Left Trigger | Hold for **Auto-Aim** — snaps robot heading toward the hub while still allowing translation |
| X | Lock wheels in an X pattern (defense against being pushed) |
| Back + Start | Zero the gyro (alliance-aware) |
| D-Pad Up | Reset odometry to the pose at the Hub |
| D-Pad Down | Reset odometry to the pose at the Outpost |

**Operator controller (port 1):**

| Input | Action |
|---|---|
| A / B / X / Y | Shoot at low / mid / high / max RPM setpoints |
| Right Trigger | Shoot using the **auto-computed RPM** for the robot's current distance to the hub |
| Right Bumper | Intake |
| Left Bumper | Outtake |
| D-Pad Up / Down | Manually jog the indexer |
| D-Pad Left / Right | Manually jog the shooter |

## Cool Features

### Auto-Aim

`commands/AutoAimCommand.java`

Holding the driver's left trigger engages auto-aim. Instead of taking over the whole drivetrain, it layers a heading controller on top of the driver's normal translation input using YAGSL's `SwerveInputStream.aim(...)`:

- The target is the hub's center point, alliance-flipped via `AllianceFlipUtil` so the same code works from either alliance station.
- The driver keeps full control of translation (moving around the field) while the robot automatically rotates to face the hub.
- The current aim target is pushed to the dashboard's `Field2d` widget (`AimTarget`) so it's visible in Shuffleboard/Elastic/AdvantageScope for debugging.
- On release, the aim behavior is cleanly disabled and the debug pose is cleared.

This means a driver can be strafing and repositioning at the same time the robot is automatically tracking the hub — no need to stop and manually line up a shot.

### Auto-Shoot / Distance-Adaptive Shot Table

`commands/ShootAndIndexCommand.java`

The main shooting command supports three ways to pick a target RPM:

1. A **fixed setpoint** (used by the A/B/X/Y "canned" shot buttons).
2. A **supplier** for dynamic goals.
3. **Automatic, distance-based lookup** (used by the operator's right trigger) — this is the "auto-shoot" feature:
   - The robot continuously computes its live distance to the hub from odometry (`SwerveSubsystem#distanceFromHubMeters`).
   - That distance is fed into an `InterpolatingDoubleTreeMap` built from a hand-tuned shot table (distance → RPM, with time-of-flight data recorded for future use), so the shooter automatically speeds up or slows down as the robot moves closer to or further from the hub.
   - Once the flywheel is within 50 RPM of the computed goal, the indexer automatically feeds the game piece into the shooter — the operator doesn't have to time the feed by hand.

There's also a standalone `AutoShoot` command that spins the shooter to a fixed goal and only starts feeding once the flywheel reaches 90% of the target speed, then runs for a fixed duration — used for simpler, one-shot autonomous shooting sequences (`ShootBalls` named command).

### Vision Pose Estimation

`subsystems/SwerveSubsystem.java`

A Limelight is mounted on the robot and configured (pipeline, camera offset, AprilTag ID filter, throttle) in `setupLimeLight()`. Every loop:

- The Limelight's robot-orientation input is kept in sync with the swerve gyro for MegaTag1 pose estimation.
- If a pose estimate comes back with low tag ambiguity (`< 0.3`) and more than one tag in view, it's fused into the drivetrain's pose estimator via `swerveDrive.addVisionMeasurement(...)`, correcting for odometry drift over a match.
- The vision-estimated pose is also pushed to `Field2d` for visualization.

This is what makes the distance-based auto-shoot and auto-aim features reliable over the course of a match — without vision correction, pure wheel odometry would drift too much for either to stay accurate.

### Alliance-Aware Field Geometry

`utils/AllianceFlipUtil.java`, `utils/FieldConstants.java`

All field landmarks (hub, trenches, towers, outpost, depot, etc.) are defined once from the blue alliance's perspective, derived directly from the official AprilTag field layout. `AllianceFlipUtil` mirrors any pose/translation/rotation across the field when the robot is on the red alliance, so auto-aim, odometry resets, and autonomous routines all work correctly regardless of which alliance station the robot starts in.

### Autonomous

Autonomous routines are authored in PathPlanner (`src/main/deploy/pathplanner/`) and selected at match time via a `SendableChooser` published to the dashboard. Named commands available to autonomous routines (registered in `RobotContainer`):

- `ShootBalls` / `ShootBallsOdom` — shoot using a fixed or odometry-driven RPM for up to 4 seconds.
- `StartIntake` — run the intake.
- `Stop` — stop the shooter and indexer.

## Building and Deploying

This is a standard WPILib Gradle project.

```bash
# Build and run unit tests
./gradlew build

# Deploy to the robot (must be on the robot's network)
./gradlew deploy

# Simulate the robot code on your desktop
./gradlew simulateJava
```

You'll need the WPILib extension/toolchain installed (see [WPILib's install guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)). The project targets team number `9658` (`.wpilib/wpilib_preferences.json`).

## Tuning

Most values a mentor or student would want to adjust live in `Constants.java`:

- **PID / feedforward gains** — per-mechanism `SmartMotorControllerConfig` (`.withClosedLoopController(...)`, `.withFeedforward(...)`).
- **Shooter RPM setpoints** — `Constants.Shooter.Setpoints`.
- **Shot table** (distance → RPM for auto-shoot) — the `shots` list at the top of `ShootAndIndexCommand.java`.
- **Swerve behavior** — module gearing/PID live in `src/main/deploy/swerve/`; max speed and starting pose are in `Constants.SwerveDrive`.
- **Limelight camera offset / AprilTag filter** — `SwerveSubsystem#setupLimeLight`.
