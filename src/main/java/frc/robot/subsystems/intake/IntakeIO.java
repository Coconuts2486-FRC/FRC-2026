package frc.robot.subsystems.intake;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends RBSIIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean pivotConnected = false;
    public boolean rollerConnected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void setRollerVelocity(double velocity) {}

  public default void setPivotVelocity(double velocity) {}

  public default void stopRoller() {}

  public default void stopPivot() {}

  public default double getPosition() {
    return 0.0;
  }

  public default void pivotToPos(double pos) {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default boolean isIntakeRollersRunning() {
    return false;
  }

  public default boolean isIntakeExtended() {
    return false;
  }
}
