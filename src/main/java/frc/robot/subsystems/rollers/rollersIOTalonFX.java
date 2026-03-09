package frc.robot.subsystems.rollers;

import static frc.robot.Constants.RobotDevices.INTAKE_ROLLER;

import com.ctre.phoenix6.hardware.TalonFX;

public class rollersIOTalonFX implements rollersIO {

  private final TalonFX rollers =
      new TalonFX(INTAKE_ROLLER.getDeviceNumber(), INTAKE_ROLLER.getCANBus());

  public rollersIOTalonFX() {}

  @Override
  public void runRollers(double speed) {
    rollers.set(speed);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
  }

  @Override
  public boolean isIntakeRollersRunning() {
    if (Math.abs(rollers.get()) > 0.02) {
      return true;
    } else {
      return false;
    }
  }
}
