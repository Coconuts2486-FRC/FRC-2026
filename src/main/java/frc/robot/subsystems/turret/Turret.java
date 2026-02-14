package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends RBSISubsystem {
  public TurretIO io;
  public double solution1;
  public double solution2;

  private double turretPosition;

  // Constructor
  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void rbsiPeriodic() {

    turretPosition =
        MathUtil.inputModulus(io.getTurretEncoderPosition(), 1, 10)
            / TurretConstants.kTurretGearRatio;

    Logger.recordOutput("Turret/Switch", readTurretSwitch());
  }

  @Override
  public void simulationPeriodic() {}

  public void aimTarget() {}

  public double wantedVelocity(double robotSpeed, double z, double distance) {

    solution1 =
        (2 * A(z, distance, robotSpeed) * robotSpeed)
            / (-B(z, distance)
                + Math.sqrt(
                    (B(z, distance) * B(z, distance))
                        - 4 * A(z, distance, robotSpeed) * C(z, distance)));
    solution2 =
        (2 * A(z, distance, robotSpeed) * robotSpeed)
            / (-B(z, distance)
                - Math.sqrt(
                    (B(z, distance) * B(z, distance))
                        - 4 * A(z, distance, robotSpeed) * C(z, distance)));

    return solution1; /*place holder */
  }

  public double A(double z, double distance, double robotSpeed) {
    return z + ((9.8 * distance * distance) / (2 * robotSpeed * robotSpeed));
  }

  public double B(double z, double distance) {
    return (2 * z * Math.cos(TurretConstants.kHoodAngle))
        - (distance * Math.sin(TurretConstants.kHoodAngle));
  }

  public double C(double z, double distance) {
    return (z * Math.cos(TurretConstants.kHoodAngle))
        - (distance * Math.sin(TurretConstants.kHoodAngle) * Math.cos(TurretConstants.kHoodAngle));
  }

  public void setVolts(double volts) {
    io.setVolts(volts);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getTurretEncoderPosition() {
    return io.getTurretEncoderPosition();
  }

  public boolean readTurretSwitch() {
    return io.readTurretSwitch();
  }

  public void stop() {
    io.stop();
  }
}
