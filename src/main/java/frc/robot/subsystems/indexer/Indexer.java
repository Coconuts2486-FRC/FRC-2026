package frc.robot.subsystems.indexer;

import frc.robot.util.RBSISubsystem;

public class Indexer extends RBSISubsystem {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  public void feederSetVelocity(double velocity) {
    io.feederSetVelocity(velocity);
  }

  public void feederStop() {
    io.feederStop();
  }

  public double getFeederspeed() {
    return io.getFeederspeed();
  }

  public void indexerSetVelocity(double velocity) {
    io.indexerSetVelocity(velocity);
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void rbsiPeriodic() {}

  @Override
  public void simulationPeriodic() {}
  
}
