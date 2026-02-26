package frc.robot.subsystems.indexer;

import static frc.robot.Constants.RobotDevices.INDEXER_ROLLER;

import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX indexer =
      new TalonFX(INDEXER_ROLLER.getDeviceNumber(), INDEXER_ROLLER.getCANBus());

  public final int[] powerPorts = {INDEXER_ROLLER.getPowerPort()};

  public IndexerIOTalonFX() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {}

  @Override
  public void setVelocity(double velocity) {
    indexer.set(velocity);
  }

  @Override
  public void indexerStop() {
    indexer.stopMotor();
  }
}
