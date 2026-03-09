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

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends RBSIIO {

  @AutoLog
  public static class TurretIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double getTurretEncoderPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  public default void aimTarget() {}

  public default void setVolts(double volts) {}

  public default void setPosition(double position) {}

  public default void stop() {}

  public default void turretEncoderPos(double pos) {}

  public default void zeroEncoder() {}

  public default double getTurretEncoderPosition() {
    return 0.0;
  }

  public default boolean readTurretSwitch() {
    return false;
  }
}
