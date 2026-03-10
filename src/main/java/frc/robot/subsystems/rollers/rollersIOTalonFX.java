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

package frc.robot.subsystems.rollers;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.rollers.rollersIO.rollersIOInputs;

public class rollersIOTalonFX implements rollersIO {

  private final TalonFX rollers =
      new TalonFX(INTAKE_ROLLER.getDeviceNumber(), INTAKE_ROLLER.getCANBus());

  private final StatusSignal<Angle> rollersPosition = rollers.getPosition();
  private final StatusSignal<AngularVelocity> rollersVelocity = rollers.getVelocity();
  private final StatusSignal<Voltage> rollersAppliedVolts = rollers.getMotorVoltage();
  private final StatusSignal<Current> rollersCurrent = rollers.getSupplyCurrent();

  /** Constructor */
  public rollersIOTalonFX() {}

  /** Update inputs */
  @Override
  public void updateInputs(rollersIOInputs inputs) {
    var rollerStatus =
        BaseStatusSignal.refreshAll(
            rollersPosition, rollersVelocity, rollersAppliedVolts, rollersCurrent);

    inputs.rollersConnected = rollerStatus.isOK();
    inputs.positionRad =
        Units.rotationsToRadians(rollersPosition.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(rollersVelocity.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.appliedVolts = rollersAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {rollersCurrent.getValueAsDouble()};
  }

  @Override
  public void runRollers(double speed) {
    rollers.set(speed);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
  }

  @Override
  public boolean isIntakeRollersRunning() {
    if (Math.abs(rollers.get()) > 0.02) {
      return true;
    } else {
      return false;
    }
  }
}
