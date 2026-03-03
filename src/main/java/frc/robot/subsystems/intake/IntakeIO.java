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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean pivotConnected = false;
    public boolean rollerConnected = false;
    public Angle pivotPositionRot = Rotations.of(0.0);
    public AngularVelocity pivotAvAngularVelocity = RotationsPerSecond.of(0.0);
    public AngularVelocity rollersAngularVelocity = RotationsPerSecond.of(0.0);
    public Voltage pivotAppliedVolts = Volts.of(0.0);
    public Voltage rollersAppliedVolts = Volts.of(0.0);
    public double[] currentAmps = new double[] {};
    public boolean releaseButton = false;
  }

  public default void setRollerPrimitiveSpeed(double velocity) {}

  public default void setPivotPrimitiveSpeed(double velocity) {}

  public default void stopRoller() {}

  public default void stopPivot() {}

  public default double getPivotPosition() {
    return 6.7;
  }

  public default void pivotToPos(double pos) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default boolean isIntakeRollersRunning() {
    return false;
  }

  public default boolean isIntakeExtended() {
    return false;
  }

  public default boolean getReleaseState() {
    return false;
  }
}
