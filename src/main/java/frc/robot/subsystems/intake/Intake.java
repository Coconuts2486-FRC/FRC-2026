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

import org.wpilib.math.controller.PIDController;
import org.wpilib.command2.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends RBSISubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // pid used for bringing intake up to stored pos
  PIDController controller =
      new PIDController(IntakeConstants.kp.get(), IntakeConstants.ki, IntakeConstants.kd);

  public Intake(IntakeIO io) {
    this.io = io;
    io.setCoast();

    /*default command has intake always come up
    unles overided by toggle of intake down in robot container */
    setDefaultCommand(Commands.run(() -> pivotUp(), this));
  }

  // ** Periodic functions
  // *************************************************************************************************** */
  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void simulationPeriodic() {}

  // ** base functions
  // ******************************************************************************************************** */

  public void setPivotPrimitiveSpeed(double speed) {
    io.setPivotPrimitiveSpeed(speed);
  }

  // Stop the pivot motion
  public void stopPivot() {
    io.stopPivot();
  }

  public void pivotDown() {

    if (io.getPivotPositionRot() > IntakeConstants.dropPosition) {
      io.setPivotPrimitiveSpeed(0.4);
    } else if (io.getPivotPositionRot() > IntakeConstants.lowerPosition) {
      io.setPivotPrimitiveSpeed(0.1);
    } else {
      io.stopPivot();
    }
  }

  public void pivotUp() {
    io.setPivotPrimitiveSpeed(
        -controller.calculate(io.getPivotPositionRot(), IntakeConstants.storedAngle));
  }

  public void pivotGoToPosition(double pos) {
    io.setPivotPrimitiveSpeed(controller.calculate(io.getPivotPositionRot(), pos));
  }

  public void printPos() {
    System.out.println(io.getPivotPositionRot());
  }

  // ** getter functions
  // ************************************************************************************************** */

  public double getPivotPosition() {
    return io.getPivotPositionRot();
  }

  public boolean isIntakeExtended() {
    return io.isIntakeExtended();
  }

  public boolean pivotAlive() {
    return inputs.pivotConnected;
  }

  // ** power port function
  // ***********************************************************************************************
  // */

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
