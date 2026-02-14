package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;

public class FlywheelIOTalonFX implements FlywheelIO {

  public static TalonFX flywheel = new TalonFX(777);

  public FlywheelIOTalonFX() {}

  @Override
  public void flywheelSetVelocity(double velocity) {
    flywheel.set(velocity);
  }

  @Override
  public void flywheelStop() {
    flywheel.stopMotor();
  }

  @Override
  public double getVelocity() {
    return flywheel.getVelocity().getValueAsDouble();
  }
}
