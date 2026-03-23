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

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Rollers extends RBSISubsystem {
  private RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;

    setDefaultCommand(Commands.run(() -> stop(), this));
  }

  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
    Logger.recordOutput("Rollers/RollersRunning", (Math.abs(inputs.velocityRadPerSec) > 0));
  }

  public void runRollers() {
    io.runRollers(0.80);
  }

  public void stop() {
    io.stop();
  }

  public boolean isIntakeRollersRunning() {
    return io.isIntakeRollersRunning();
  }

  public boolean isRollersAlive() {
    return inputs.rollersConnected;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
