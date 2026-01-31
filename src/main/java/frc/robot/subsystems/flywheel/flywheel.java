package frc.robot.subsystems.flywheel;

import frc.robot.util.RBSISubsystem;

public class flywheel extends RBSISubsystem {

  private flywheelIO io;

  // Constructor
  public flywheel(flywheelIO io) {
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
  public void rbsiPeriodic() {}

  @Override
  public void simulationPeriodic() {}
}
