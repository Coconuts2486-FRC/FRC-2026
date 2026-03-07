// Copyright (c) 2026 FRC-2486
// https://github.com/Coconuts2486-FRC
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

package frc.robot.subsystems.indexer;

import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RBSIEnum.CTREPro;

public class IndexerIOTalonFX implements IndexerIO {

  // Declare Hardware
  private final TalonFX indexer =
      new TalonFX(INDEXER_ROLLER.getDeviceNumber(), INDEXER_ROLLER.getCANBus());
  public final int[] POWER_PORTS = {INDEXER_ROLLER.getPowerPort()};

  private final StatusSignal<Angle> indexerPosition = indexer.getPosition();
  private final StatusSignal<AngularVelocity> indexerVelocity = indexer.getVelocity();
  private final StatusSignal<Voltage> indexerAppliedVolts = indexer.getMotorVoltage();
  private final StatusSignal<Current> indexerCurrent = indexer.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /** Constructor */
  public IndexerIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (kShooterIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kShooterOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = kShooterOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = kShooterOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kShooterClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = kShooterClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = kShooterClosedLoopRampPeriod;
    // Apply the open- and closed-loop ramp configuration for current smoothing
    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);
    // set Motion Magic Velocity settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    // Apply the configurations to the Shooter motors
    PhoenixUtil.tryUntilOk(5, () -> indexer.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, indexerPosition, indexerVelocity, indexerAppliedVolts, indexerCurrent);
    indexer.optimizeBusUtilization();
  }

  /** Update inputs */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        indexerPosition, indexerVelocity, indexerAppliedVolts, indexerCurrent);
    inputs.positionRad =
        Units.rotationsToRadians(indexerPosition.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(indexerVelocity.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.appliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {indexerCurrent.getValueAsDouble()};
  }

  /**
   * Set Velocity
   *
   * @param velocity Desired indexer velocity
   */
  @Override
  public void setVelocity(double velocity) {
    // Can use Motion Magic and/or TorqueFOC control here!
    indexer.set(velocity);
  }

  /** Stop the indexer */
  @Override
  public void indexerStop() {
    indexer.stopMotor();
  }
}
