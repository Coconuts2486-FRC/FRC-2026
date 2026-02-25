package frc.robot.subsystems.feeder;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO extends RBSIIO {

  @AutoLog
  public static class FeederIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};

    public boolean feederConnected = false;
  }

  public default void feederSetVelocity(double velocity) {}

  public default void feederStop() {}

  public default double getFeederspeed() {
    return 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}
}
