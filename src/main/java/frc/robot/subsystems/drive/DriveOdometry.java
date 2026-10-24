// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.driverstation.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.TimeUtil;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public final class DriveOdometry extends VirtualSubsystem {
  private static final int MAX_VISION_MEASUREMENTS_PER_LOOP = 2;

  // Declare the io and inputs
  private final Drive drive;
  private final Imu imu;
  private final Module[] modules;
  private final List<BusRefreshGroup> busRefreshGroups;
  private final StatusCode[] moduleRefreshStatuses;

  private record BusRefreshGroup(
      String name, BaseStatusSignal[] signals, int[] moduleIndexes) {}

  // Per-cycle cached objects (to avoid repeated allocations)
  private final SwerveModulePosition[] odomPositions = new SwerveModulePosition[4];
  private SwerveModulePosition[] previousYawPositions;
  private Rotation2d lastOdometryYaw = Rotation2d.kZero;
  private double lastYawSampleTime = Double.NaN;
  private double lastWheelYawRate = 0.0;

  // Checking whether this is a REPLAY
  private boolean isReplayActive = Logger.hasReplaySource();

  /** Constructor */
  public DriveOdometry(Drive drive, Imu imu, Module[] modules) {
    this.drive = drive;
    this.imu = imu;
    this.modules = modules;
    moduleRefreshStatuses = new StatusCode[modules.length];

    Map<String, List<BaseStatusSignal>> signalsByBus = new LinkedHashMap<>();
    Map<String, List<Integer>> moduleIndexesByBus = new LinkedHashMap<>();
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      for (var entry : modules[moduleIndex].getBulkRefreshSignalsByBus().entrySet()) {
        if (entry.getValue().length == 0) continue;
        signalsByBus
            .computeIfAbsent(entry.getKey(), ignored -> new ArrayList<>())
            .addAll(Arrays.asList(entry.getValue()));
        moduleIndexesByBus.computeIfAbsent(entry.getKey(), ignored -> new ArrayList<>()).add(moduleIndex);
      }
    }
    busRefreshGroups = new ArrayList<>(signalsByBus.size());
    signalsByBus.forEach(
        (busName, signals) ->
            busRefreshGroups.add(
                new BusRefreshGroup(
                    busName,
                    signals.toArray(new BaseStatusSignal[0]),
                    moduleIndexesByBus.get(busName).stream().mapToInt(Integer::intValue).toArray())));
  }

  /**
   * Priority value for this virtual subsystem
   *
   * <p>See `frc.robot.util.VirtualSubsystem` for a description of the suggested values for various
   * virtual subsystems.
   */
  @Override
  protected int getPeriodPriority() {
    return -20;
  }

  /** Periodic function to read inputs */
  @Override
  public void rbsiPeriodic() {
    final long loopStartNanos = System.nanoTime();
    final long bulkRefreshStartNanos = System.nanoTime();
    Arrays.fill(moduleRefreshStatuses, StatusCode.OK);
    StatusCode bulkRefreshStatus = StatusCode.OK;
    for (BusRefreshGroup group : busRefreshGroups) {
      StatusCode busStatus = BaseStatusSignal.refreshAll(group.signals());
      Logger.recordOutput("Odometry/BusRefreshStatus/" + group.name(), busStatus.toString());
      if (bulkRefreshStatus.isOK() && !busStatus.isOK()) {
        bulkRefreshStatus = busStatus;
      }
      for (int moduleIndex : group.moduleIndexes()) {
        if (moduleRefreshStatuses[moduleIndex].isOK()) {
          moduleRefreshStatuses[moduleIndex] = busStatus;
        }
      }
    }
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      modules[moduleIndex].setBulkRefreshStatus(moduleRefreshStatuses[moduleIndex]);
    }
    final long bulkRefreshEndNanos = System.nanoTime();

    final long lockWaitStartNanos = System.nanoTime();
    Drive.odometryLock.lock();
    final long lockAcquiredNanos = System.nanoTime();
    long moduleInputsEndNanos = lockAcquiredNanos;
    long estimatorStartNanos = lockAcquiredNanos;
    int sampleCount = 0;
    try {
      final var imuInputs = imu.getInputs();
      Logger.recordOutput("Odometry/UsingWheelHeading", !imuInputs.connected);

      // Read refreshed module telemetry and drain each odometry queue once per loop.
      for (var module : modules) {
        module.periodic();
      }
      moduleInputsEndNanos = System.nanoTime();
      estimatorStartNanos = moduleInputsEndNanos;

      // ----------------------------------------------------------------------
      // Pure SIM (not replaying a log): use sim pose/yaw
      // ----------------------------------------------------------------------
      if (Constants.isPureSim()) {
        final double now = TimeUtil.now();

        // Keep buffers alive
        drive.poseBufferAddSample(now, drive.getSimPose());
        drive.yawBuffersAddSample(now, drive.getSimYawRad(), drive.getSimYawRateRadPerSec());

        // Coast state uses "now" + current module positions
        drive.updateDisabledCoastState(
            RobotState.isEnabled(),
            RobotState.isDisabled(),
            now,
            drive.getSimYawRateRadPerSec(),
            drive.getModulePositions());

        return;
      }

      // ----------------------------------------------------------------------
      // DISABLED (REAL only): keep both estimator history and external buffers alive so delayed
      // vision measurements can still be applied while the robot is carried or repositioned.
      // ----------------------------------------------------------------------
      if (RobotState.isDisabled() && !isReplayActive) {
        final double now = TimeUtil.now();

        // keep yaw buffers alive
        SwerveModulePosition[] currentPositions = drive.getModulePositions();
        Rotation2d yaw =
            yawForSample(
                now, imuInputs.connected, imuInputs.yawPositionRad, currentPositions);
        drive.yawBuffersAddSample(
            now,
            yaw.getRadians(),
            imuInputs.connected ? imuInputs.yawRateRadPerSec : lastWheelYawRate);

        // Coast state from "now" + current module positions
        drive.updateDisabledCoastState(
            RobotState.isEnabled(),
            RobotState.isDisabled(),
            now,
            imuInputs.connected ? imuInputs.yawRateRadPerSec : lastWheelYawRate,
            currentPositions);

        drive.poseEstimatorUpdateWithTime(now, yaw, currentPositions);

        // Keep the external vision-alignment buffer on the same timestamp and pose.
        drive.poseBufferAddSample(now, drive.getPose());
        drive.setGyroDisconnectedAlert(!imuInputs.connected);
        return;
      }

      // ----------------------------------------------------------------------
      // Canonical timestamp queue from module[0]
      // ----------------------------------------------------------------------
      final double[] ts = modules[0].getOdometryTimestamps();
      final int n = (ts == null) ? 0 : ts.length;
      sampleCount = n;

      // Always keep yaw buffers "alive" even if no samples
      if (n == 0) {
        if (Constants.getMode() != Mode.REPLAY) {
          final double now = TimeUtil.now();
          drive.yawBuffersAddSample(
              now,
              lastOdometryYaw.getRadians(),
              imuInputs.connected ? imuInputs.yawRateRadPerSec : 0.0);

          // Coast state update (no per-sample positions available; use current)
          drive.updateDisabledCoastState(
              RobotState.isEnabled(),
              RobotState.isDisabled(),
              now,
              imuInputs.connected ? imuInputs.yawRateRadPerSec : 0.0,
              drive.getModulePositions());
        }
        drive.setGyroDisconnectedAlert(!imuInputs.connected);
        return;
      }

      // Cache module histories once
      final SwerveModulePosition[][] modHist = new SwerveModulePosition[4][];
      for (int m = 0; m < 4; m++) {
        modHist[m] = modules[m].getOdometryPositions();
      }

      // ----------------------------------------------------------------------
      // Determine YAW queue availability (everything exists and lines up)
      // ----------------------------------------------------------------------
      final boolean hasYawQueue =
          imuInputs.connected
              && imuInputs.odometryYawTimestamps != null
              && imuInputs.odometryYawPositionsRad != null
              && imuInputs.odometryYawTimestamps.length == imuInputs.odometryYawPositionsRad.length
              && imuInputs.odometryYawTimestamps.length > 0;

      final double[] yawTs = hasYawQueue ? imuInputs.odometryYawTimestamps : null;
      final double[] yawPos = hasYawQueue ? imuInputs.odometryYawPositionsRad : null;

      // Determine index alignment
      boolean yawIndexAligned = false;
      if (hasYawQueue && yawTs.length >= n) {
        yawIndexAligned = true;
        final double eps = 1e-3; // 1ms
        for (int i = 0; i < n; i++) {
          if (Math.abs(yawTs[i] - ts[i]) > eps) {
            yawIndexAligned = false;
            break;
          }
        }
      }

      // If yaw not aligned, pre-fill yaw buffers once and interpolate later
      if (hasYawQueue && !yawIndexAligned) {
        drive.yawBuffersFillFromQueue(yawTs, yawPos);
      } else if (imuInputs.connected) {
        final double now = TimeUtil.now();
        drive.yawBuffersAddSample(now, imuInputs.yawPositionRad, imuInputs.yawRateRadPerSec);
      }

      // ----------------------------------------------------------------------
      // Replay each odometry sample
      // ----------------------------------------------------------------------
      final double[] lastDist = new double[4];
      boolean haveLastDist = false;
      final boolean logDebug = Constants.tuningMode;

      for (int i = 0; i < n; i++) {
        final double t = ts[i];

        // Build module positions at sample i (clamp defensively)
        for (int m = 0; m < 4; m++) {
          final SwerveModulePosition[] hist = modHist[m];
          if (hist == null || hist.length == 0) {
            odomPositions[m] = modules[m].getPosition();
          } else if (i < hist.length) {
            odomPositions[m] = hist[i];
          } else {
            odomPositions[m] = hist[hist.length - 1];
          }
        }

        // Determine yaw at this timestamp
        double yawRad = imuInputs.yawPositionRad;
        if (hasYawQueue) {
          if (yawIndexAligned) {
            yawRad = yawPos[i];
            drive.yawBuffersAddSampleIndexAligned(t, yawTs, yawPos, i);
          } else {
            yawRad = drive.yawBufferSampleOr(t, imuInputs.yawPositionRad);
          }
        }
        Rotation2d yaw = yawForSample(t, imuInputs.connected, yawRad, odomPositions);
        yawRad = yaw.getRadians();
        if (!imuInputs.connected) {
          drive.yawBuffersAddSample(t, yawRad, lastWheelYawRate);
        }

        // Coast state update IN REPLAY TIMEBASE
        // Yaw rate: if you have a buffered rate, use it; otherwise imuInputs.yawRateRadPerSec is
        // ok.
        drive.updateDisabledCoastState(
            RobotState.isEnabled(),
            RobotState.isDisabled(),
            t,
            imuInputs.connected ? imuInputs.yawRateRadPerSec : lastWheelYawRate,
            odomPositions);

        // Detailed replay logging is expensive at the odometry sample rate. In tuning mode, retain
        // only the newest sample from each robot loop.
        if (logDebug && i == n - 1) {
          Logger.recordOutput("Odometry/Debug/timestamp", t);
          Logger.recordOutput("Odometry/Debug/now", TimeUtil.now());
          if (i > 0) {
            Logger.recordOutput("Odometry/Debug/timeNowDiff", t - ts[i - 1]);
          }
          Logger.recordOutput("Odometry/Debug/replay_t", t);
          Logger.recordOutput("Odometry/Debug/replay_yawRad", yawRad);
        }

        // Module distance deltas (valid within batch)
        for (int m = 0; m < 4; m++) {
          final SwerveModulePosition pos = odomPositions[m];
          final double dist = pos.distance;

          if (logDebug && i == n - 1) {
            Logger.recordOutput("Odometry/Debug/mod" + m + "_distanceMeters", dist);
            Logger.recordOutput("Odometry/Debug/mod" + m + "_angleRad", pos.angle.getRadians());
          }

          if (haveLastDist) {
            final double delta = dist - lastDist[m];
            if (logDebug && i == n - 1) {
              Logger.recordOutput("Odometry/Debug/mod" + m + "_deltaMeters", delta);
            }
          }

          lastDist[m] = dist;
        }
        haveLastDist = true;

        // Feed estimator at this historical timestamp
        drive.poseEstimatorUpdateWithTime(t, yaw, odomPositions);

        // Maintain pose history in SAME timebase as estimator
        drive.poseBufferAddSample(t, drive.getPose());
      }

      drive.setGyroDisconnectedAlert(!imuInputs.connected);

    } finally {
      final long visionApplyStartNanos = System.nanoTime();
      int visionMeasurementsProcessed = 0;
      long visionApplyEndNanos;
      try {
        visionMeasurementsProcessed =
            drive.applyQueuedVisionMeasurements(MAX_VISION_MEASUREMENTS_PER_LOOP);
        Logger.recordOutput("Odometry/Robot", drive.getPose());
      } finally {
        visionApplyEndNanos = System.nanoTime();
        Drive.odometryLock.unlock();
      }

      final long loopEndNanos = System.nanoTime();
      Logger.recordOutput(
          "Odometry/Timing/BulkRefreshMS", (bulkRefreshEndNanos - bulkRefreshStartNanos) / 1e6);
      Logger.recordOutput(
          "Odometry/Timing/LockWaitMS", (lockAcquiredNanos - lockWaitStartNanos) / 1e6);
      Logger.recordOutput(
          "Odometry/Timing/ModuleInputsAndQueueDrainMS",
          (moduleInputsEndNanos - lockAcquiredNanos) / 1e6);
      Logger.recordOutput(
          "Odometry/Timing/EstimatorMS", (visionApplyStartNanos - estimatorStartNanos) / 1e6);
      Logger.recordOutput(
          "Odometry/Timing/VisionApplyMS", (visionApplyEndNanos - visionApplyStartNanos) / 1e6);
      Logger.recordOutput("Odometry/Timing/TotalMS", (loopEndNanos - loopStartNanos) / 1e6);
      Logger.recordOutput("Odometry/SamplesProcessed", sampleCount);
      Logger.recordOutput("Odometry/VisionMeasurementsProcessed", visionMeasurementsProcessed);
      Logger.recordOutput("Odometry/BulkRefreshStatus", bulkRefreshStatus.toString());
    }
  }

  private Rotation2d yawForSample(
      double timestamp, boolean imuConnected, double imuYawRad, SwerveModulePosition[] positions) {
    if (imuConnected && Double.isFinite(imuYawRad)) {
      lastOdometryYaw = Rotation2d.fromRadians(imuYawRad);
      lastWheelYawRate = 0.0;
    } else if (previousYawPositions != null) {
      double delta = drive.wheelHeadingDeltaRadians(previousYawPositions, positions);
      if (Double.isFinite(delta)) {
        lastOdometryYaw = lastOdometryYaw.plus(Rotation2d.fromRadians(delta));
        double dt = timestamp - lastYawSampleTime;
        lastWheelYawRate = dt > 1e-6 ? delta / dt : 0.0;
      }
    }
    previousYawPositions = positions.clone();
    lastYawSampleTime = timestamp;
    return lastOdometryYaw;
  }
}
