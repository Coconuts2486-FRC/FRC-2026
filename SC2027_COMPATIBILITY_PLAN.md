# 2027 SystemCore Compatibility Plan

This document tracks the work required to make the imported 2027 SystemCore project compile while
preserving the team's robot configurations, generated tuner constants, mechanisms, autos, CAN IDs,
and team number.

## Compatibility policy

- Use the Java 25 JDK installed with WPILib and keep the WPILib/GradleRIO artifacts on the imported
  `2027.0.0-alpha-6` baseline.
- Keep Commands V2 while migrating the existing command-based robot. Do not introduce a Commands V3
  rewrite as part of compatibility work.
- Do not add a 2026 vendor binary to the 2027 runtime unless the vendor explicitly declares it
  compatible with SystemCore and the installed WPILib alpha.
- Preserve integrations without compatible dependencies behind adapters or separate source sets
  rather than deleting the implementation.
- Compile after each API family so root causes are not hidden by javac's error limit.

## 1. Stabilize the 2027 build environment

- [x] Configure Gradle's Java toolchain for Java 25.
- [x] Make both Gradle wrapper launchers select the JDK in the selected WPILib installation before
  Gradle initializes toolchain discovery.
- [x] Retain GradleRIO `2027.0.0-alpha-6`, Gradle 9.4.1, and the imported SystemCore target.
- [x] Keep Commands V2 as the command framework.
- [ ] Establish `./gradlew compileJava` as a passing checkpoint after the dependency and API work
  below removes the remaining source errors.

The local WPILib directory is named `2027_alpha5`, but its Maven repository and JDK contain the
alpha-6 GradleRIO/WPILib artifacts used by this project. The directory name therefore remains the
installer-selected lookup key; dependency versions remain pinned to alpha-6.

## 2. Reconstruct the dependency configuration

- [x] Restore the AdvantageKit `akit-autolog` annotation processor using the version declared in
  `vendordeps/AdvantageKit.json`.
- [x] Declare Jackson Databind directly instead of relying on a removed vendor library's transitive
  dependency.
- [x] Replace the project's single Lombok-generated getter with explicit Java getters, avoiding an
  otherwise unnecessary annotation processor.
- [x] Enable desktop support for tests and simulation.
- [x] Check official compatibility information for NavX, YAGSL, and Choreo. No dependency compatible
  with the alpha-5/6 SystemCore baseline is currently available, so none was added.
- [ ] Verify whether URCL's declared 2026 version supports the imported SystemCore runtime.
- [x] Confirm `akit-autolog` is present on Gradle's annotation-processor classpath.
- [x] Confirm AdvantageKit generates all `*InputsAutoLogged` classes during `compileJava`.

Current imported vendor baselines include AdvantageKit alpha-4, Autopilot alpha-6, PathPlanner
alpha-3, Phoenix 6 alpha-1, REVLib alpha-6, PhotonVision alpha-2, Commands V2, and URCL 2026. The
NavX, YAGSL, and Choreo vendordeps present in the 2026 tree were not supplied by the 2027 importer.
WPILib's SystemCore compatibility matrix explicitly lists ChoreoLib as unavailable for alpha-5/6.
Studica marks its old NavX vendordep as deprecated for 2027, and the former YAGSL repository has been
archived as `YAGSL_old`. These implementations must remain isolated until supported replacements are
published.

References:

- [WPILib SystemCore compatibility matrix](https://github.com/wpilibsuite/SystemcoreTesting#third-party-library-compatibility-by-wpilib-version)
- [Studica NavX release information](https://github.com/Studica-Robotics/NavX)
- [Archived YAGSL repository](https://github.com/Yet-Another-Software-Suite/YAGSL_old)

## Current checkpoint

`./gradlew --version` now reports Java 25.0.2 for both the launcher and daemon. Dependency reports
resolve successfully, including Jackson 2.21.1 and AdvantageKit `akit-autolog` 27.0.0-alpha-4.
`compileJava` now reaches javac without a toolchain or dependency-resolution failure and stops on the
expected step 3 WPILib API conversions. AdvantageKit generated types are now generated successfully.
Choreo, NavX, and YAGSL source paths have been retained as comments because compatible dependencies
are unavailable; Phoenix/Pigeon and PathPlanner remain the active configurations. The remaining
compilation failures are confined to later WPILib API conversion work.

## 3. Correct WPILib importer conversions

- Replace `ChassisSpeeds` with `ChassisVelocities` and `SwerveModuleState` with
  `SwerveModuleVelocity`, including method and field semantics.
- Move `DCMotor` references from `math.system.plant` to `math.system`.
- Move `Alert` references to `driverstation.Alert` and use the standalone `driverstation.Alliance`.
- Replace Xbox, PS4, and PS5 command controllers with their `CommandNiDs...Controller` equivalents.
- Adapt or remove obsolete `BuiltInAccelerometer`, LiveWindow, and HAL resource-reporting calls.

## 4. Restore AdvantageKit generated sources

- Verify annotation processing produces the module, rollers, shooter, feeder, indexer, and intake
  `*InputsAutoLogged` types.
- Address genuine AdvantageKit API changes only after generated-source failures are removed.
- Keep generated build metadata behavior separate from robot-generated tuner constants.

## 5. Resolve vendor integrations

- Update REV imports and API calls, starting with the moved `ControlType` declaration.
- Compile CTRE-generated constants and drivetrain implementations without regenerating team files.
- Check PhotonVision, PathPlanner, Autopilot, and URCL against their imported versions.
- Restore compatible NavX, YAGSL, and Choreo dependencies or isolate those optional implementations.

## 6. Compile in subsystem order

1. Core utilities, constants, and the `first.Main` entry point.
2. AdvantageKit IO interfaces and generated sources.
3. IMU and accelerometer.
4. Drivetrain and odometry.
5. Vision.
6. Mechanisms.
7. Autonomous routines and controller bindings.
8. All production sources, followed by all test sources.

## 7. Migrate and run tests

- Convert tests to the 2027 WPILib packages and velocity types.
- Run focused utility, IMU, drivetrain, odometry, and vision tests.
- Run the complete Gradle `test` and `build` tasks.

## 8. Validate SystemCore packaging

- Verify `first.Main` is the correct executable entry point.
- Build and inspect the shaded deployment JAR, service files, and deploy resources.
- Remove remaining RoboRIO-only deployment assumptions.
- Review the final diff for changes to team number, CAN IDs, generated tuner constants, autos, and
  mechanism behavior.

## Verification commands

Run these in increasing scope as their corresponding migration stages become available:

```sh
./gradlew compileJava
./gradlew compileTestJava
./gradlew test
./gradlew build
```
