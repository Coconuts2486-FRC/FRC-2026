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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends RBSISubsystem {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Max values are for rotations a second
  PIDController controller =
      new PIDController(IntakeConstants.kp.get(), IntakeConstants.ki, IntakeConstants.kd);

  /** Constructor */
  public Intake(IntakeIO io) {
    this.io = io;

    setDefaultCommand(Commands.run(() -> pivotUp(), this));
  }

  /** Simulation periodic function */
  @Override
  public void simulationPeriodic() {}

  /** Periodic function */
  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setPivotPrimitiveSpeed(double speed) {
    io.setPivotPrimitiveSpeed(speed);
  }

  /** Stop the pivot motion */
  public void stopPivot() {
    io.stopPivot();
  }

  public void pivotDown() {
    if (io.getPivotPosition() > IntakeConstants.dropPostion) {
      io.setPivotPrimitiveSpeed(-0.4);
    } else {
      io.stopPivot();
      io.setCoast();
    }
  }

  public void pivotUp() {
    io.setPivotPrimitiveSpeed(
        controller.calculate(io.getPivotPosition(), IntakeConstants.storedAngle));
  }

  public void pivotGoToPosition(double pos) {
    io.setPivotPrimitiveSpeed(controller.calculate(io.getPivotPosition(), pos));
  }

  public double getPivotPosition() {
    return io.getPivotPosition();
  }

  public boolean isIntakeExtended() {
    return io.isIntakeExtended();
  }

  public boolean pivotAlive() {
    return inputs.pivotConnected;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
