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

package frc.robot.subsystems.feeder;

import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class FeederIOTalonFX implements FeederIO {

  // Declare Hardware
  private final TalonFX feeder =
      new TalonFX(FEEDER_ROLLER.getDeviceNumber(), FEEDER_ROLLER.getCANBus());
  public final int[] POWER_PORTS = {FEEDER_ROLLER.getPowerPort()};

  private final StatusSignal<Angle> feederPosition = feeder.getPosition();
  private final StatusSignal<AngularVelocity> feederVelocity = feeder.getVelocity();
  private final StatusSignal<Voltage> feederAppliedVolts = feeder.getMotorVoltage();
  private final StatusSignal<Current> feederCurrent = feeder.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /** Constructor */
  public FeederIOTalonFX() {
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
    PhoenixUtil.tryUntilOk(5, () -> feeder.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, feederPosition, feederVelocity, feederAppliedVolts, feederCurrent);
    feeder.optimizeBusUtilization();
  }

  /** Update Inputs */
  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            feederPosition, feederVelocity, feederAppliedVolts, feederCurrent);
    inputs.feederAlive = status.isOK();
    inputs.positionRad =
        Units.rotationsToRadians(feederPosition.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(feederVelocity.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.appliedVolts = feederAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {feederCurrent.getValueAsDouble()};
  }

  /** Motor Control Functions ********************************************** */
  /**
   * Set the motor voltage
   *
   * @param volts Voltage to which to set the motor
   */
  @Override
  public void setVoltage(double volts) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(volts);
    m_request.withEnableFOC(isCTREPro);
    feeder.setControl(m_request);
  }

  /**
   * Set the motor velocity
   *
   * @param velocityRadPerSec The velocity to which to set the motor
   */
  @Override
  public void setVelocity(double velocityRadPerSec) {
    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    m_request.withEnableFOC(isCTREPro);
    feeder.setControl(m_request.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  /**
   * Set the motor percent
   *
   * @param percent The percent to which to set the motor
   */
  @Override
  public void setPercent(double percent) {
    // create a Motion Magic DutyCycle request, voltage output
    final MotionMagicDutyCycle m_request = new MotionMagicDutyCycle(percent);
    m_request.withEnableFOC(isCTREPro);
    feeder.setControl(m_request);
  }

  @Override
  public void setFeederVelocity(double velocity) {
    feeder.set(velocity);
  }

  /** Stop the feeder */
  @Override
  public void stopFeeder() {
    feeder.stopMotor();
    feeder.setControl(new MotionMagicDutyCycle(0.));
  }

  /** Getter Functions ***************************************************** */
  /** Get feeder running state */
  @Override
  public boolean isFeederRunning() {
    return (Math.abs(feeder.get()) > 0.1);
  }
}
