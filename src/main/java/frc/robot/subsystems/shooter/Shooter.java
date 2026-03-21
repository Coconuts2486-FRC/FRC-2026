// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.computations.FieldRelativeShooterSolver;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends RBSISubsystem {

  // Declare IO
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine sysId;
  private double targetMetersPerSecond = 0.0;
  private double shooterOffset = 0.0;

  FieldRelativeShooterSolver.FieldShotSolution solution;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configureGains(kPreal, 0.0, kDreal, kSreal, kVreal, kAreal);
        break;
      case SIM:
      default:
        io.configureGains(kPsim, 0.0, kDsim, kSsim, kVsim, kAsim);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  /** Periodic function -- inherits timing logic from RBSISubsystem */
  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/targetspeed", targetMetersPerSecond);
    Logger.recordOutput(
        "Shooter/currentSpeed",
        Math.abs(
            ((Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec) / 60)
                    / ShooterConstants.kShooterGearRatio)
                * ShooterConstants.flywheelCircumfrence));
    Logger.recordOutput("Shooter/atSpeed", shooterAtSpeed());
    Logger.recordOutput("Shooter/ShooterOffset", shooterOffset);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void set(double set) {
    targetMetersPerSecond = 1;

    io.set(set);
  }

  public void runTargetVelocity(
      Pose3d robotPose,
      Transform3d launcherTransform,
      Pose3d targetPose,
      Translation2d platformVelocity) {

    solution =
        FieldRelativeShooterSolver.solve(
            robotPose, launcherTransform, targetPose, platformVelocity);

    runVelocity(0);

    System.out.println(solution.v0());
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocity) {

    targetMetersPerSecond = velocity;

    double speed =
        (velocity / ShooterConstants.flywheelCircumfrence)
            * ShooterConstants.kShooterGearRatio
            * -1;

    io.setVelocity(speed);

    // Log Shooter setpoint
  }

  /** Stops the Shooter. */
  public void stop() {
    targetMetersPerSecond = 0.0;
    io.stop();
  }

  public void incrementOffset(double change) {
    shooterOffset = shooterOffset + change;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Mechanism/Shooter")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean shooterAtSpeed() {
    if (targetMetersPerSecond == 0.0) return false;
    double currentMetersPerSecond =
        ((Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec) / 60)
                / ShooterConstants.kShooterGearRatio)
            * ShooterConstants.flywheelCircumfrence;
    return true;
  }

  public boolean leaderAlive() {
    return inputs.leaderAlive;
  }

  public boolean followerAlive() {
    return inputs.followerAlive;
  }

  public double shooterOffset() {
    return shooterOffset;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
