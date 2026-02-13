package frc.robot.subsystems.flywheel;

import frc.robot.util.RBSISubsystem;

public class Flywheel extends RBSISubsystem {

  private FlywheelIO io;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void flywheelSetVelocity(double Velocity) {
    io.flywheelSetVelocity(Velocity);
  }

  public void flywheelStop() {
    io.flywheelStop();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  @Override
  public void rbsiPeriodic() {}

  @Override
  public void simulationPeriodic() {}
}
