// Copyright 2026 FRC 2486
// https://github.com/Coconuts2486-FRC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.simulation.DriverStationSim;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.imu.ImuIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveTest {
  private static final double EPSILON = 1e-9;
  private static Drive sharedDrive;

  @BeforeAll
  static void createDriveOnce() {
    assertTrue(HAL.initialize(500, 0));
    sharedDrive = new Drive(new Imu(new ImuIOSim()));
  }

  @BeforeEach
  void setupHal() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void yawRateUsesShortestAngleAcrossWraparound() {
    double previousYaw = Math.toRadians(179.0);
    double yaw = Math.toRadians(-179.0);

    assertEquals(Math.toRadians(20.0), Drive.calculateYawRate(previousYaw, yaw, 0.1), EPSILON);
  }

  @Test
  void invalidGyroYawUsesLastKnownFiniteHeading() {
    assertTrue(DriveOdometry.isValidYaw(true, 1.25));
    assertFalse(DriveOdometry.isValidYaw(false, 1.25));
    assertFalse(DriveOdometry.isValidYaw(true, Double.NaN));

    assertEquals(1.25, DriveOdometry.selectYawRad(true, 1.25, 0.5), EPSILON);
    assertEquals(0.5, DriveOdometry.selectYawRad(false, Double.NaN, 0.5), EPSILON);
  }

  @Test
  void pathPlannerStartResetsWhenVisionIsMissingOrEstimatorIsUninitialized() {
    Pose2d pathStart = new Pose2d(4.0, 6.0, Rotation2d.kZero);

    assertEquals(
        Drive.PathPlannerStartAction.RESET_TO_PATH_START,
        Drive.determinePathPlannerStartAction(
            new Pose2d(6.0, 6.0, Rotation2d.kZero), pathStart, false, 0.5));
    assertEquals(
        Drive.PathPlannerStartAction.RESET_TO_PATH_START,
        Drive.determinePathPlannerStartAction(Pose2d.kZero, pathStart, true, 0.5));
  }

  @Test
  void pathPlannerStartUsesNearbyVisionAndBlocksDistantVision() {
    Pose2d pathStart = new Pose2d(4.0, 6.0, Rotation2d.kZero);

    assertEquals(
        Drive.PathPlannerStartAction.USE_VISION_POSE,
        Drive.determinePathPlannerStartAction(
            new Pose2d(4.4, 6.0, Rotation2d.kZero), pathStart, true, 0.5));
    assertEquals(
        Drive.PathPlannerStartAction.BLOCK_AUTO,
        Drive.determinePathPlannerStartAction(
            new Pose2d(4.6, 6.0, Rotation2d.kZero), pathStart, true, 0.5));
  }

  @Test
  void poseBufferAccessorsAreSafeWhenEmptyAndInterpolateWhenPopulated() {
    Drive drive = sharedDrive;

    assertTrue(Double.isNaN(drive.getPoseBufferOldestTime()));
    assertTrue(Double.isNaN(drive.getPoseBufferNewestTime()));
    assertTrue(drive.getPoseAtTime(1.0).isEmpty());

    drive.poseBufferAddSample(1.0, new Pose2d(1.0, 0.0, Rotation2d.kZero));
    drive.poseBufferAddSample(2.0, new Pose2d(3.0, 0.0, Rotation2d.kZero));
    assertEquals(2.0, drive.getPoseAtTime(1.5).orElseThrow().getX(), EPSILON);
  }

  @Test
  void modulePeriodicUsesCommonOdometryPrefix() {
    ModuleIO fakeIo =
        new ModuleIO() {
          @Override
          public void updateInputs(ModuleIOInputs inputs) {
            inputs.driveConnected = true;
            inputs.turnConnected = true;
            inputs.turnEncoderConnected = true;
            inputs.odometryTimestamps = new double[] {1.0, 2.0, 3.0};
            inputs.odometryDrivePositionsRad = new double[] {4.0, 5.0};
            inputs.odometryTurnPositions =
                new Rotation2d[] {Rotation2d.kZero, Rotation2d.kCCW_Pi_2, Rotation2d.kPi};
          }
        };
    Module module = new Module(fakeIo, 0);

    assertDoesNotThrow(module::periodic);
    assertEquals(2, module.getOdometryPositions().length);
  }

  @Test
  void disabledCoastDoesNotCountBaselineSampleAsStationary() {
    Drive drive = sharedDrive;
    SwerveModulePosition[] stationaryPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero)
        };

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drive.updateDisabledCoastState(true, false, 1.0, 0.0, stationaryPositions);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    drive.updateDisabledCoastState(false, true, 1.02, 0.0, stationaryPositions);
    for (int i = 0; i < 9; i++) {
      drive.updateDisabledCoastState(false, true, 1.30 + i * 0.02, 0.0, stationaryPositions);
    }
    assertTrue(drive.isDisabledCoast(1.48));

    drive.updateDisabledCoastState(false, true, 1.50, 0.0, stationaryPositions);
    assertFalse(drive.isDisabledCoast(1.50));
  }
}
