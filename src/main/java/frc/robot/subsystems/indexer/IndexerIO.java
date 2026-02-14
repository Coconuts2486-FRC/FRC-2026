package frc.robot.subsystems.indexer;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO extends RBSIIO {

  @AutoLog
  public static class IndexerIOInputs {
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

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void indexerSetVelocity(double velocity) {}

  public default void indexerStop() {}
}
