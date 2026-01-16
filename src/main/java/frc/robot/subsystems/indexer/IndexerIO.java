package frc.robot.subsystems.indexer;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO extends RBSIIO {

  @AutoLog
  public static class CoralScorerIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void indexerSetVolts(double volts) {}

  public default void indexerSetVelocity(double velocity) {}

  public default void indexerSetPercent(double percent) {}

  public default void indexerStop() {}
}
