package frc.robot.subsystems.Turret;

import frc.robot.Constants.turretConstants;
import frc.robot.util.RBSISubsystem;

public class Turret extends RBSISubsystem {
  public TurretIO io;
  public double solution1;
  public double solution2;

  // Constructor
  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void rbsiPeriodic() {}

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
    return (2 * z * Math.cos(turretConstants.hoodAngle))
        - (distance * Math.sin(turretConstants.hoodAngle));
  }

  public double C(double z, double distance) {
    return (z * Math.cos(turretConstants.hoodAngle))
        - (distance * Math.sin(turretConstants.hoodAngle) * Math.cos(turretConstants.hoodAngle));
  }

  public void setVolts(double volts) {
    io.setVolts(volts);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getPosition() {
    return io.getPosition();
  }

  public void stop() {
    io.stop();
  }
}
