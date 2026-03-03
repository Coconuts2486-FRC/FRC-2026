package frc.robot.subsystems.feeder;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO extends RBSIIO {

  @AutoLog
  public static class FeederIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};

    public boolean feederConnected = false;
  }

  public default void setFeederVelocity(double velocity) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}

  public default void stopFeeder() {}

  public default double getFeederspeed() {
    return 0.0;
  }

  public default boolean isFeederRunning() {
    return false;
  }

  public default void updateInputs(FeederIOInputs inputs) {}
}
