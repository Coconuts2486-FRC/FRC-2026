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

package frc.robot.subsystems.intake;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {

  // Declare Hardware
  private final TalonFX pivot =
      new TalonFX(INTAKE_PIVOT.getDeviceNumber(), INTAKE_PIVOT.getCANBus());
  private final TalonFX rollers =
      new TalonFX(INTAKE_ROLLER.getDeviceNumber(), INTAKE_ROLLER.getCANBus());

  public final int[] powerPorts = {INTAKE_PIVOT.getPowerPort(), INTAKE_ROLLER.getPowerPort()};

  private final CANcoder pivotEncoder =
      new CANcoder(INTAKE_ENCODER.getDeviceNumber(), INTAKE_ENCODER.getCANBus());

  private final DigitalInput releaseButton = new DigitalInput(INTAKE_RELEASE);

  /** Constructor */
  public IntakeIOTalonFX() {

    // Configure motors and encoders

    // Current limiting

    // FUSE cancoder onto pivot motor

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    // roller
    rollerConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // cancoder
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;

    // applying
    PhoenixUtil.tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(cancoderConfig));
    PhoenixUtil.tryUntilOk(5, () -> rollers.getConfigurator().apply(rollerConfig));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // checks the status of pivot
    var pivotStatus =
        BaseStatusSignal.refreshAll(
            pivot.getStatorCurrent(),
            pivot.getPosition(),
            pivot.getVelocity(),
            pivot.getMotorVoltage());

    // checks the status of roller
    var rollerStatus =
        BaseStatusSignal.refreshAll(
            rollers.getStatorCurrent(),
            rollers.getPosition(),
            rollers.getVelocity(),
            rollers.getMotorVoltage());

    inputs.pivotConnected = pivotStatus.isOK();
    inputs.rollerConnected = rollerStatus.isOK();
    inputs.pivotPositionRot = pivot.getPosition().getValue();
    inputs.pivotAvAngularVelocity = pivot.getVelocity().getValue();
    inputs.rollersAngularVelocity = rollers.getVelocity().getValue();
    inputs.releaseButton = getReleaseState();
    inputs.rollersAppliedVolts = rollers.getMotorVoltage().getValue();
    inputs.pivotAppliedVolts = pivot.getMotorVoltage().getValue();
    inputs.currentAmps =
        new double[] {
          pivot.getSupplyCurrent().getValueAsDouble(), rollers.getSupplyCurrent().getValueAsDouble()
        };
  }

  /** Set the coast mode of the mechanism as COAST */
  @Override
  public void setCoast() {
    pivot.setNeutralMode(NeutralModeValue.Coast);
    rollers.setNeutralMode(NeutralModeValue.Coast);
  }

  /** Set the coast mode of the mechanism as BRAKE */
  @Override
  public void setBrake() {
    pivot.setNeutralMode(NeutralModeValue.Brake);
    rollers.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Set the mechanism angular velocity in physical units ***************** */
  /**
   * Set the primitive speed of the pivot
   *
   * @param speed Primitive speed in the range -1.0 to 1.0
   */
  @Override
  public void setPivotPrimitiveSpeed(double speed) {
    pivot.set(speed);
  }

  /**
   * Set the primitive speed of the pivot
   *
   * @param speed Primitive speed in the range -1.0 to 1.0
   */
  @Override
  public void setRollerPrimitiveSpeed(double speed) {
    rollers.set(speed);
  }

  @Override
  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void stopRoller() {
    rollers.stopMotor();
  }

  /** Getter functions ***************************************************** */
  /**
   * Get the intake rollers running boolean
   *
   * @return Whether the rollers are running
   */
  @Override
  public boolean isIntakeRollersRunning() {
    if (Math.abs(rollers.get()) > 0.02) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Get the intake extended boolean
   *
   * @return Whether the intake is extended
   */
  @Override
  public boolean isIntakeExtended() {
    return (pivotEncoder.getAbsolutePosition().getValueAsDouble()
        > (IntakeConstants.dropPostion - 0.05));
  }

  @Override
  public double getPivotPosition() {
    // The encoder returns position in units of rotations
    return pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Get the current pressed state of the intake release button
   *
   * <p>NOTE: Because of how DIO ports work, when the switch is not pressed, the port reads "high"
   * or "true". This function returns whether the button is pressed, so negates the output of the
   * DIO read function.
   *
   * @return true if the button is pressed, false otherwise
   */
  @Override
  public boolean getReleaseState() {
    return !releaseButton.get();
  }
}
