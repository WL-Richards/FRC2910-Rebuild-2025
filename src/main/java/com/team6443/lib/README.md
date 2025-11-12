# Team 6443 Shared Library (com.team6443.lib)

## Overview
Reusable building blocks for robot years: configurations, IO abstractions, simulation backends, factories, and utilities. Encourages interface-driven subsystems with swappable hardware/sim implementations.

## Package Map
- `can`: CAN device helpers and status logging.
- `config`: Strongly typed configuration objects (camera, encoders, motors, swerve, robot, odometry, wrappers).
- `controllers`: Driver and operator input wrappers (`XboxControllerImplementation`) and controller interfaces.
- `encoders`: IO interfaces and hardware/sim support for encoders.
- `factories`: Builders for motors and encoders (e.g., `TalonFXFactory`).
- `logging`: Logging contracts to integrate with AdvantageKit.
- `math`: Utilities like `ConcurrentTimeInterpolatableBuffer`.
- `mechanics`: Mechanical helpers (e.g., `MultistageGearBox`).
- `motors`: IO abstractions and backends for motors (`hardware/` TalonFXIO, `sim/` TalonFXSimIO).
- `network`: Misc network helpers.
- `phoenix6`: CTRE-specific utilities.
- `subsystems`: Base subsystem types and IO contracts (e.g., drivetrain IO/inputs, servo motor subsystems).
- `subsystems/simulation`: Simulation backends and visualizations (MapleSim swerve, elevator, viz).

## Core Patterns
- IO Interfaces: Define device/drive contracts (e.g., `subsystems/drive/DrivetrainIO`).
- Inputs Snapshots: Struct-like state containers (e.g., `subsystems/drive/DrivetrainInputs`).
- Config Objects: Builder-style, immutable-in-practice configuration (e.g., `config/subsystems/drive/DrivetrainConfiguration`, `DrivetrainSimConfiguration`).
- Hardware vs. Sim: Provide `hardware/` and `sim/` implementations; app layer chooses via a factory.
- Factories: Centralize construction and CTRE configuration (e.g., `factories/motors/TalonFXFactory`).

## Drivetrain Example
- Configuration: Provide `DrivetrainConfiguration` and `DrivetrainSimConfiguration`.
- IO: Implement `DrivetrainIO` (hardware via CTRE swerve; sim via MapleSim + Notifier thread).
- Subsystem Composition: App layer composes IO and config (`frc2025/subsystems/drive/DrivetrainSubsystem`), logs via AdvantageKit.
- Visualization: `subsystems/simulation/visualizations/DrivetrainVisualization`.

## Simulation Details
MapleSim swerve integrates via `subsystems/simulation/drive/MapleSimSwerveDrivetrain` and runs at `kSimLoopPeriodMS`. Sim world pose is synchronized to CTRE odometry for realistic field-relative states.

## Extending the Library
1. Create an IO interface and Inputs snapshot for the device/subsystem.
2. Implement `hardware/` and (optionally) `sim/` backends.
3. Add a config object and a factory method if construction needs centralization.
4. Keep logging prefixes and units consistent; wire through AdvantageKit `Logger`.

## Conventions
- Logging: Prefix with subsystem path, e.g., `Subsystems/<name>/...`.
- Units: Use SI units internally; document conversions at boundaries.
- Determinism: Minimize allocations and avoid blocking in periodic loops.

## Run/Use
The library is consumed by the year app (e.g., `frc2025`). Build and simulate with the standard WPILib Gradle tasks (`./gradlew build`, `./gradlew simulateJava`).

