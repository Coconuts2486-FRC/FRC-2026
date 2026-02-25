package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

public class FeederIOTalonFX implements FeederIO {

  private static TalonFX Feeder = new TalonFX(882);
  private static TalonFX feeder = new TalonFX(560);

  public FeederIOTalonFX() {}

  @Override
  public void updateInputs(FeederIOInputs inputs) {}

  @Override
  public void feederSetVelocity(double velocity) {
    feeder.set(velocity);
  }

  @Override
  public void feederStop() {
    feeder.stopMotor();
  }

  @Override
  public void FeederSetVelocity(double velocity) {
    Feeder.set(velocity);
  }

  @Override
  public void FeederStop() {}
}
