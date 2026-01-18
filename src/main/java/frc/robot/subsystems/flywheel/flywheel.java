package frc.robot.subsystems.flywheel;

import frc.robot.util.RBSISubsystem;

public class flywheel extends RBSISubsystem {

  private flywheelIO io;

  public void flywheel(flywheelIO io) {
    this.io = io;
  }

  public void flywheelSetVolts(double volts) {
    io.flywheelSetVolts(volts);
  }

  public void flywheelSpeed(double speed) {
    io.flywheelSpeed(speed);
  }

  public void flywheelStop() {
    io.flywheelStop();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
