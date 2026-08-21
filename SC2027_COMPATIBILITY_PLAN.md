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
- [x] Establish `./gradlew compileJava` as a passing checkpoint after the dependency and API work
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
- [x] Verify whether URCL's declared 2026 version supports the imported SystemCore runtime. Its
  native driver does not list a Systemcore platform, so startup remains commented and REVLib's
  built-in logging remains active.
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
`compileJava` now reaches javac without a toolchain or dependency-resolution failure. The WPILib API
conversion and AdvantageKit generation stages are complete. Choreo, NavX, and YAGSL source paths
have been retained as comments because compatible dependencies are unavailable; Phoenix/Pigeon and
PathPlanner remain the active configurations. Production and test compilation now pass. The Phoenix
6 mechanism API migration, vendor isolation, focused tests, full test suite, and shaded package
build are complete.

## 3. Correct WPILib importer conversions

- [x] Replace `ChassisSpeeds` with `ChassisVelocities` and `SwerveModuleState` with
  `SwerveModuleVelocity`, including method and field semantics.
- [x] Move `DCMotor` references from `math.system.plant` to `math.system` and replace the removed
  `LinearSystemId` motor factory with `Models`.
- [x] Move `Alert` references to `driverstation.Alert` and use the split `MatchState`, `RobotState`,
  and `DriverStationErrors` APIs.
- [x] Replace Xbox, PS4, and PS5 command controllers with their `CommandNiDs...Controller`
  equivalents.
- [x] Adapt obsolete `BuiltInAccelerometer`, LiveWindow, HAL resource reporting, FPGA time,
  NetworkTables options, PWM motor, and math utility calls.

## 4. Restore AdvantageKit generated sources

- [x] Verify annotation processing produces the module, rollers, shooter, feeder, indexer, intake,
  and IMU
  `*InputsAutoLogged` types.
- [x] Update the AdvantageKit power-distribution initialization for the SystemCore CAN bus ID.
- [x] Keep generated build metadata behavior separate from robot-generated tuner constants.

## 5. Resolve vendor integrations

- [x] Update REV imports and API calls, starting with the moved `ControlType` declaration.
- [x] Compile CTRE-generated constants and drivetrain implementations without regenerating team
  files, and migrate mechanism control calls to Phoenix 6 control requests and status signals.
- [x] Check PhotonVision, PathPlanner, Autopilot, and URCL against their imported versions.
- [x] Isolate NavX, YAGSL, Choreo, and URCL implementations for which compatible Systemcore
  dependencies are unavailable.

## 6. Compile in subsystem order

1. [x] Core utilities, constants, and the `first.Main` entry point.
2. [x] AdvantageKit IO interfaces and generated sources.
3. [x] IMU and accelerometer.
4. [x] Drivetrain and odometry.
5. [x] Vision.
6. [x] Mechanisms.
7. [x] Autonomous routines and controller bindings.
8. [x] All production sources, followed by all test sources.

## 7. Migrate and run tests

- [x] Convert tests to the 2027 WPILib packages and velocity types.
- [x] Run focused utility, controller, constants, IMU, drivetrain, odometry, accelerometer, and
  vision tests.
- [x] Run the complete Gradle `test` and `build` tasks.

## 8. Validate SystemCore packaging

- [x] Verify `first.Main` is the correct executable entry point.
- [x] Build and inspect the shaded deployment JAR, service files, source/vendor backup, and deploy
  resources.
- [x] Remove remaining RoboRIO-only deployment assumptions from active code and deploy guidance.
- [x] Review the final diff for changes to team number, CAN IDs, generated tuner constants, autos,
  and mechanism behavior.

## Completed verification

- `git diff --check` passes; this project defines no Spotless or other formatter task.
- `./gradlew compileJava` and `./gradlew compileTestJava` pass.
- The focused test selection passes, including drivetrain, odometry queue, vision, IMU,
  accelerometer, controller, constants, and utility behavior.
- `./gradlew test` and `./gradlew build` pass all 36 tests and produce the thin and shaded JARs.
- The shaded JAR manifest names `first.Main`, merges service files, and contains source/vendor
  backups. Gradle deploys `src/main/deploy` separately to `/home/systemcore/deploy`.
- Team 2486, robot CAN assignments, generated Pinchy/George/Compbot tuner files, intake setpoint
  values, and autonomous deploy files are unchanged from the committed migration checkpoint. The
  intake setpoint and IO names now explicitly identify rotations without changing their values.

## Verification commands

Run these in increasing scope as their corresponding migration stages become available:

```sh
./gradlew compileJava
./gradlew compileTestJava
./gradlew test
./gradlew build
```
