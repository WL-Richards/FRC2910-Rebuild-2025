# FRC 2025 Robot App (com.team6443.frc2025)

## Overview
Year-specific robot code that composes subsystems and configuration from the shared `com.team6443.lib` library. The app cleanly separates configuration, IO, commands, and subsystems, and supports both real hardware and desktop simulation.

## Directory Structure
- Entry: `Main.java`, `Robot.java`
- Wiring: `RobotContainer.java`
- Config: `config/` (robot IDs, per-robot hardware/swerve/elevator configs)
- Constants: `constants/` (runtime mode, state constants, build constants)
- Subsystems: `subsystems/` (factory + subsystem code; hardware and sim IO live here for year code)
- Commands: `commands/` (e.g., driving commands)
- Simulation Hooks: `subsystems/drive/DrivetrainIOSim.java`

```
src/main/java/com/team6443/frc2025/
├── Main.java
├── Robot.java
├── RobotContainer.java
├── RobotState.java
├── SimulatedRobotState.java
├── commands/
│   └── drive/DriveWithHeadingCommand.java
├── config/
│   ├── RobotConfig.java
│   ├── RobotID.java
│   └── robots/nautilus/... (swerve + elevator + cameras)
├── constants/
│   ├── RobotRuntimeConstants.java
│   └── RobotStateConstants.java
└── subsystems/
    ├── SubsystemFactory.java
    ├── drive/
    │   ├── DrivetrainSubsystem.java
    │   ├── DrivetrainIOHardware.java
    │   └── DrivetrainIOSim.java
    └── elevator/ElevatorSubsystem.java
```

## Key Classes
- `Main`: Starts the WPILib robot (`Robot`).
- `Robot`: Extends AdvantageKit `LoggedRobot`, runs the scheduler, updates `RobotState` and logs.
- `RobotContainer`: Creates subsystems, binds commands (e.g., default drivetrain drive-with-heading).
- `SubsystemFactory`: Chooses hardware vs. sim implementations based on `RobotRuntimeConstants.kCurrentRuntimeMode`.
- `RobotRuntimeConstants`: Decides runtime mode (REAL/SIM), robot ID, and active `RobotConfig`.
- `config/robots/nautilus/*`: Hardware IDs and configurations for robot “Nautilus” (swerve modules, elevator, cameras, physical params).

## Runtime & Simulation
- Runtime mode auto-detects: roboRIO → REAL, desktop → SIM.
- Drivetrain sim uses `DrivetrainIOSim` with `lib.config.subsystems.drive.DrivetrainSimConfiguration` and MapleSim backend.
- Hardware path uses `DrivetrainIOHardware` with CTRE Phoenix 6 swerve and Pigeon2.

## Patterns
- Subsystems depend on IO interfaces from `lib` (e.g., drivetrain uses `lib.subsystems.drive.DrivetrainIO` and `DrivetrainInputs`).
- Configuration objects live under `frc2025/config` and `lib/config/**` and are passed into subsystem constructors.
- Commands are thin and use input suppliers from controller wrappers in `lib`.

## Add or Modify a Subsystem
1. Define config under `frc2025/config/...` and/or `lib/config/...`.
2. Implement IO (hardware and optional sim) using `lib` IO patterns.
3. Build a `Subsystem` that owns the IO and logs via AdvantageKit.
4. Register construction in `SubsystemFactory`, set default command in `RobotContainer`.

## Build, Simulate, Deploy
- Desktop sim: `./gradlew simulateJava`
- Build (checks + jar): `./gradlew build`
- Deploy to roboRIO: `./gradlew deploy`

AdvantageKit logs are produced automatically via `LoggedRobot` and `Logger` calls throughout subsystems and IO layers.

## Vendordeps
Phoenix 6, WPILib NewCommands, AdvantageKit, and MapleSim are included under `vendordeps/` and managed by GradleRIO.

