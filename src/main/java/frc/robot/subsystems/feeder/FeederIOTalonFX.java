package frc.robot.subsystems.feeder;

import static frc.robot.Constants.RobotDevices.FEEDER_ROLLER;

import com.ctre.phoenix6.hardware.TalonFX;

public class FeederIOTalonFX implements FeederIO {

  private final TalonFX feeder =
      new TalonFX(FEEDER_ROLLER.getDeviceNumber(), FEEDER_ROLLER.getCANBus());
  public final int[] powerPorts = {FEEDER_ROLLER.getPowerPort()};

  public FeederIOTalonFX() {}

  @Override
  public void updateInputs(FeederIOInputs inputs) {}

  @Override
  public void setFeederVelocity(double velocity) {
    feeder.set(velocity);
  }

  @Override
  public boolean isFeederRunning() {
    if (feeder.get() > 0.02) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void stopFeeder() {
    feeder.stopMotor();
  }
}
