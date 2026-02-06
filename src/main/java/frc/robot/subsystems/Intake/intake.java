package frc.robot.subsystems.Intake;

import frc.robot.util.RBSISubsystem;

public class intake extends RBSISubsystem {
  private intakeIO io;

  public intake(intakeIO io) {
    this.io = io;
  }

  @Override
  public void simulationPeriodic() {}

  public void setRollerVelocity(double velocity) {
    io.setRollerVelocity(velocity);
  }

  public void setPivotVelocity(double velocity) {
    io.setPivotVelocity(velocity);
  }

  public void pivotDown() {
    if (io.getPosition() < 9) {
      io.setPivotVelocity(0.5);
      io.setRollerVelocity(0.65);
    } else {
      io.stopPivot();
      io.setRollerVelocity(0.65);
    }
  }

  public void pivotUp() {
    io.goToPosition(5);
  }

  public void pivotGoToPosition(double pos) {
    io.goToPosition(pos);
  }

  public double getPosition() {
    return io.getPosition();
  }

  public void stopRoller() {
    io.stopRoller();
  }

  public void stopPivot() {
    io.stopPivot();
  }
}
