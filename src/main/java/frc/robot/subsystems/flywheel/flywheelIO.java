package frc.robot.subsystems.flywheel;

import frc.robot.util.RBSIIO;

public interface flywheelIO extends RBSIIO {

  public default void flywheelSetVolts(double volts) {}

  public default void flywheelSpeed(double speed) {}

  public default void flywheelStop() {}
}
