package frc.robot.subsystems.indexer;

import frc.robot.util.RBSISubsystem;

public class Indexer extends RBSISubsystem {
  private final IndexerIO io;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void indexerSetVolts(double volts) {
    io.indexerSetVolts(volts);
  }

  public void indexerSetVelocity(double velocity) {
    io.indexerSetVelocity(velocity);
  }

  public void indexerSetPercent(double percent) {
    io.indexerSetPercent(percent);
  }

  public void stop() {}

  @Override
  public void rbsiPeriodic() {}

  @Override
  public void simulationPeriodic() {}
}
