package frc.robot.subsystems.Intake;

import frc.robot.util.RBSIIO;

public interface intakeIO extends RBSIIO {

  public default void rollerSetVolts(double volts) {}

  public default void pivotSetVolts(double volts) {}

  public default void setPosition(double position) {}

  public default void setRollerSpeed(double speed) {}

  public default void stopRoller() {}

  public default void stopPivot() {}

  public default double getPosition() {
    return 0.0;
  }
}
