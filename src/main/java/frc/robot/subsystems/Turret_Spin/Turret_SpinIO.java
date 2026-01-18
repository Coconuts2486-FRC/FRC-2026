package frc.robot.subsystems.Turret_Spin;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

import com.thethriftybot.wrappers.DriverStationWrapper.defaultDriversStation;

public interface Turret_SpinIO extends RBSIIO {

  @AutoLog
  public static class Turret_SpinIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void aimTarget(){}

  public default void setVolts(double volts) {}

  public default void setPosition(double position) {}

  public default double getPosition() {
    return 0.0;
  }

  public default void stop() {}
}
