package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOTalonFX implements IndexerIO {

  private static TalonFX indexer = new TalonFX(882);
  private static TalonFX feeder = new TalonFX(560);

  public IndexerIOTalonFX() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {}

  @Override
  public void feederSetVelocity(double velocity) {
    feeder.set(velocity);
  }

  @Override
  public void feederStop() {
    feeder.stopMotor();
  }

  @Override
  public void indexerSetVelocity(double velocity) {
    indexer.set(velocity);
  }

  @Override
  public void indexerStop() {}
}
