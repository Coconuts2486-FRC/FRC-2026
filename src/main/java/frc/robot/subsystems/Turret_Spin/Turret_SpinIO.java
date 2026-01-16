package frc.robot.subsystems.Turret_Spin;

import frc.robot.util.RBSIIO;

public interface Turret_SpinIO extends RBSIIO {

  public default void setVolts(double volts) {}

  public default void setPosition(double position) {}

  public default double getPosition() {
    return 0.0;
  }

  public default void stop() {}
}
