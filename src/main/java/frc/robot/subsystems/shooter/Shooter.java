// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.computations.FieldRelativeShooterSolver;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends RBSISubsystem {

  // Declare IO
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double targetMetersPerSecond = 0.0;
  private double currentMetersPerSecond;

  FieldRelativeShooterSolver.FieldShotSolution solution;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  /** Periodic function -- inherits timing logic from RBSISubsystem */
  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/TargetSpeed", targetMetersPerSecond);

    Logger.recordOutput("Shooter/CurrentSpeed", currentMetersPerSecond);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocity) {

    targetMetersPerSecond = velocity;

    double speed =
        (velocity / ShooterConstants.flywheelCircumfrence)
            * ShooterConstants.kShooterGearRatio
            * -1;

    io.setVelocity(speed);
  }

  /** Stops the Shooter. */
  public void stop() {
    targetMetersPerSecond = 0.0;
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Mechanism/Shooter")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public boolean shooterAtSpeed() {
    if (targetMetersPerSecond == 0.0) return false;
    currentMetersPerSecond =
        ((Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec) / 60)
                / ShooterConstants.kShooterGearRatio)
            * ShooterConstants.flywheelCircumfrence;
    return Math.abs(currentMetersPerSecond) >= Math.abs(targetMetersPerSecond * 0.6);
  }

  public boolean leaderAlive() {
    return inputs.leaderAlive;
  }

  public boolean followerAlive() {
    return inputs.followerAlive;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
