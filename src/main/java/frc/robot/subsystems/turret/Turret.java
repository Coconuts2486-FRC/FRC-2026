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

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends RBSISubsystem {
  public TurretIO io;
  public double solution1;
  public double solution2;

  private double turretPosition;
  private boolean lastSwitch = false;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /** Constructor */
  public Turret(TurretIO io) {
    this.io = io;
  }

  // TODO: Should be a ProfiledPIDController!!!!
  PIDController turretPIDController =
      new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);

  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    // // boolean current = readTurretSwitch();

    // turretPosition =
    //     MathUtil.inputModulus(io.getTurretEncoderPosition(), 1, 10)
    //         / TurretConstants.kTurretGearRatio;

    // Logger.recordOutput("Turret/Is In Position?", readTurretSwitch());

    // // if (current && !lastSwitch) {
    // //   io.zeroEncoder();
    // // }

    // // lastSwitch = current;
  }

  @Override
  public void simulationPeriodic() {}

  public void aimTarget() {}

  /** Functions***************** */
  public void setVolts(double volts) {
    io.setVolts(volts);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getTurretEncoderPosition() {
    return io.getTurretEncoderPosition();
  }

  public boolean turretAlive() {
    return inputs.turretAlive;
  }

  public double simplifiedTurretPosition() {
    return MathUtil.inputModulus(io.getTurretEncoderPosition(), 1, 10)
        / TurretConstants.kTurretGearRatio;
  }

  public void rotateToPosition(double pos) {
    io.setPosition(turretPIDController.calculate(io.getTurretEncoderPosition(), pos));
  }

  public boolean readTurretSwitch() {

    return !io.readTurretSwitch();
  }

  public void stop() {
    io.stop();
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
