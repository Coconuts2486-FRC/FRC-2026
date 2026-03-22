// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RBSIEnum.CTREPro;

public class ShooterIOTalonFX implements ShooterIO {

  // Define the leader / follower motors from the Ports section of RobotContainer
  private final TalonFX leader =
      new TalonFX(SHOOTER_LEADER.getDeviceNumber(), SHOOTER_LEADER.getCANBus());
  private final TalonFX follower =
      new TalonFX(SHOOTER_FOLLOWER.getDeviceNumber(), SHOOTER_FOLLOWER.getCANBus());

  public final int[] POWER_PORTS = {SHOOTER_LEADER.getPowerPort(), SHOOTER_FOLLOWER.getPowerPort()};
  // IMPORTANT: Include here all devices listed above that are part of this mechanism!

  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();

  private final StatusSignal<Angle> followerPosition = follower.getPosition();
  private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
  private final StatusSignal<Voltage> followerAppliedVolts = follower.getMotorVoltage();

  private final StatusSignal<Current> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = follower.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /** Constructor */
  public ShooterIOTalonFX() {
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
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));
    // If follower rotates in the opposite direction, set "MotorAlignmentValue" to Opposed
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition, followerVelocity, followerAppliedVolts, followerCurrent);

    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);

    inputs.leaderAlive = leaderStatus.isOK();
    inputs.followerAlive = followerStatus.isOK();

    inputs.positionRad =
        Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / kShooterGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / kShooterGearRatio;
    inputs.velocityMetersPerSec = inputs.velocityRadPerSec * -ShooterConstants.flywheelCircumfrence;
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts).withEnableFOC(isCTREPro));
  }

  @Override
  public void setVelocity(double velocityRotationsPerSecond) {
    // create a Motion Magic Velocity request, voltage output
    final VelocityVoltage m_request = new VelocityVoltage(0);
    m_request.withEnableFOC(isCTREPro);
    leader.setControl(m_request.withVelocity(velocityRotationsPerSecond));
  }

  @Override
  public void set(double speed) {
    leader.set(speed * -1);
  }

  @Override
  public void setPercent(double percent) {
    leader.setControl(new DutyCycleOut(percent).withEnableFOC(isCTREPro));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public double get() {
    return leader.get();
  }

  /**
   * Set the gains of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   * @param kS Static gain
   * @param kV Velocity gain
   */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
  }

  /**
   * Set the gains of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
  }
}
