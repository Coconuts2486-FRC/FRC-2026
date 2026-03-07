package frc.robot.subsystems.indexer;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOTalonFX implements IndexerIO {

  private final TalonFX indexer =
      new TalonFX(INDEXER_ROLLER.getDeviceNumber(), INDEXER_ROLLER.getCANBus());
  public final int[] POWER_PORTS = {INDEXER_ROLLER.getPowerPort()};

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /** Constructor */
  public IndexerIOTalonFX() {
    // Add a whole bunch of motor configuration here
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Add the input updates here
  }

  /** Set Velocity */
  @Override
  public void setVelocity(double velocity) {
    indexer.set(velocity);
  }

  /** Stop the indexer */
  @Override
  public void indexerStop() {
    indexer.stopMotor();
  }
}
