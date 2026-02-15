package frc.robot.subsystems.turret;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO extends RBSIIO {

  @AutoLog
  public static class TurretIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void aimTarget() {}

  public default void setVolts(double volts) {}

  public default void setPosition(double position) {}

  public default void stop() {}

  public default void turretEncoderPos(double pos) {}

  public default void zeroEncoder() {}

  public default double getTurretEncoderPosition() {
    return 0.0;
  }

  public default boolean readTurretSwitch() {
    return false;
  }
}
