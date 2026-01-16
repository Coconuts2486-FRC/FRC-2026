package frc.robot.subsystems.Intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface intakeIO extends RBSIIO {

  @AutoLog
  public static class intakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void rollerSetVolts(double volts) {}

  public default void pivotSetVolts(double volts) {}

  public default void setPosition(double position) {}

  public default void setRollerVelocity(double velocity) {}

  public default void stopRoller() {}

  public default void stopPivot() {}

  public default double getPosition() {
    return 0.0;
  }
}
