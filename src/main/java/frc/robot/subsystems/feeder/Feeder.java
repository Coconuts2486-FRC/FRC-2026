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

//** periodic functions **************************************************************************************************** */

  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  @Override
  public void simulationPeriodic() {}

//** base functions ********************************************************************************************************** */

  //runs feeder at set velocity, function accesed by default command directly what changes the speed
  //remember changing this value will change the regression at time of az north regression speed 0.5
  public void runFeeder() {
    io.setFeederVelocity(0.5);
  }

  //stops feeder motor completely
  public void stopFeeder() {
    io.stopFeeder();
  }

//** getter functions **************************************************************************************************** */

  //if speed is greater than 10% returns true
  public boolean isFeederRunning() {
    return io.isFeederRunning();
  }

  //checks to make sure the feeder is returning data to make sure the can is good
  public boolean isFeederAlive() {
    return inputs.feederAlive;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
