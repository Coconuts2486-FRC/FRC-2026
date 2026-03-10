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

import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Feeder extends RBSISubsystem {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  @Override
  public void simulationPeriodic() {}

  public void runFeeder() {
    io.setFeederVelocity(0.5);
  }

  public void stopFeeder() {
    io.stopFeeder();
  }

  public double getFeederspeed() {
    return io.getFeederspeed();
  }

  public boolean isFeederRunning() {
    return io.isFeederRunning();
  }

  public boolean isFeederAlive() {
    return inputs.feederAlive;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
