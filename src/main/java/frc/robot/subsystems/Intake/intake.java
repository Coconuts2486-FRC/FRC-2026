package frc.robot.subsystems.Intake;

import frc.robot.util.RBSISubsystem;

public class intake extends RBSISubsystem {
  private intakeIO io;

  // Constructor
  public intake(intakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  public void rollerSetVolts(double volts) {
    io.rollerSetVolts(volts);
  }

  public void pivotSetVolts(double volts) {
    io.pivotSetVolts(volts);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public double getPosition() {
    return io.getPosition();
  }

  public void setRollerVelocity(double velocity) {
    io.setRollerVelocity(velocity);
  }

  public void stopRoller() {
    io.stopRoller();
  }

  public void stopPivot() {
    io.stopPivot();
  }
}
