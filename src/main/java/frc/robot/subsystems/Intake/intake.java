package frc.robot.subsystems.intake;

import frc.robot.util.RBSISubsystem;

public class Intake extends RBSISubsystem {
  private IntakeIO io;

  public Intake(IntakeIO io) {
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

  public void pivotToPos(double pos) {
    io.pivotToPos(pos);
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
