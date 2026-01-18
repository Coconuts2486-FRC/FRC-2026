package frc.robot.subsystems.Turret_Spin;

import frc.robot.util.RBSISubsystem;

public class Turret_Spin extends RBSISubsystem {
  public Turret_SpinIO io;

  public void Turret_Spin(Turret_SpinIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  public void aimTarget() {}

  public double wantedVelocity(double robotSpeed, double z, double distance) {
    return 0.0;
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
