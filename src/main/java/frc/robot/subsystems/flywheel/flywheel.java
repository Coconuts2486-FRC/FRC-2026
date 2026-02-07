package frc.robot.subsystems.flywheel;

import frc.robot.util.RBSISubsystem;

public class flywheel extends RBSISubsystem {

  private flywheelIO io;

  public void flywheel(flywheelIO io) {
    this.io = io;
  }

  public void flywheelSetVelocity(double Velocity) {
    io.flywheelSetVelocity(Velocity);
  }

  public void flywheelStop() {
    io.flywheelStop();
  }

  public double getVelocity(){
    return io.getVelocity();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}


}
