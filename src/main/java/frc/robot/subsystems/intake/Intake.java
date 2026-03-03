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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.RBSISubsystem;

public class Intake extends RBSISubsystem {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Max values are for rotations a second
  PIDController controller =
      new PIDController(IntakeConstants.kp.get(), IntakeConstants.ki, IntakeConstants.kd);

  /** Constructor */
  public Intake(IntakeIO io) {
    this.io = io;

    setDefaultCommand(
        Commands.run(() -> pivotUp(), this).alongWith(Commands.run(() -> stopRollers())));
  }

  /** Simulation periodic function */
  @Override
  public void simulationPeriodic() {}

  /** Periodic function */
  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);

    // If in disabled mode, check the "release button" related to motor state
    if (DriverStation.isDisabled()) {
      if (io.getReleaseState()) {
        // Button pressed: coast
        io.setCoast();
      } else {
        // Otherwise: brake
        io.setBrake();
      }
    } else {
      io.setCoast();
    }
  }

  /**
   * Prints the encoder position
   *
   * <p>temporary testing function
   */
  public void print() {
    System.out.println(io.getPivotPosition());
  }

  /** Mechanism setting functions ****************************************** */
  /**
   * Set the roller primitive speed
   *
   * @param speed Primitive speed value between -1.0 and 1.0
   */
  public void setRollerPrimitiveSpeed(double speed) {
    io.setRollerPrimitiveSpeed(speed);
  }

  /**
   * Set the pivot primitive speed
   *
   * @param speed Primitive speed value between -1.0 and 1.0
   */
  public void setPivotPrimitiveSpeed(double speed) {
    io.setPivotPrimitiveSpeed(speed);
  }

  /** Run the rollers at a pre-determined primitive speed */
  public void runRollers() {
    io.setRollerPrimitiveSpeed(IntakeConstants.kRollerPrimitiveSpeed);
  }

  /** Stop the rollers */
  public void stopRollers() {
    io.stopRoller();
  }

  /** Stop the pivot motion */
  public void stopPivot() {
    io.stopPivot();
  }

  /**
   * Put the pivot down
   *
   * <p>gives the intake a little push but then lets it fall down and be free while intaking
   */
  public void pivotDown() {
    if (io.getPivotPosition() > IntakeConstants.dropPostion) {
      io.setPivotPrimitiveSpeed(-0.4);
      // io.setRollerVelocity(0.65);
    } else {
      io.stopPivot();
    }
  }

  /**
   * Bring the pivot back up
   *
   * <p>brings pivot up with pid while running intake motors still, stopping them when in position
   */
  public void pivotUp() {
    if (io.getPivotPosition() < IntakeConstants.storedAngle + 0.05
        && io.getPivotPosition() > IntakeConstants.storedAngle - 0.05) {
      io.setPivotPrimitiveSpeed(
          controller.calculate(io.getPivotPosition(), IntakeConstants.storedAngle));

      io.stopRoller();
    } else {

      io.setPivotPrimitiveSpeed(
          controller.calculate(io.getPivotPosition(), IntakeConstants.storedAngle));
      io.setRollerPrimitiveSpeed(0.65);
    }
  }

  /**
   * Move the pivot to a specific position
   *
   * @param pos Anglular position to which to move the pivot
   */
  public void pivotGoToPosition(double pos) {
    io.setPivotPrimitiveSpeed(controller.calculate(io.getPivotPosition(), pos));
  }

  /** Getter functions ***************************************************** */
  /**
   * Get the pivot position
   *
   * @return Pivot position (currently in rotations -- should change this to degrees!)
   */
  public double getPivotPosition() {
    return io.getPivotPosition();
  }

  /**
   * Get whether the intake rollers are running
   *
   * @return Rollers running boolean
   */
  public boolean isIntakeRollersRunning() {
    return io.isIntakeRollersRunning();
  }

  /**
   * Get whether the intake is extended
   *
   * @return Intake extended boolean
   */
  public boolean isIntakeExtended() {
    return io.isIntakeExtended();
  }

  /**
   * Get whether the rollers are alive
   *
   * @return rollersAlive boolean
   */
  public boolean rollersAlive() {
    return inputs.rollerConnected;
  }

  /**
   * Get whether the pivot is alive
   *
   * @return pivotAlive boolean
   */
  public boolean pivotAlive() {
    return inputs.pivotConnected;
  }
}
