package frc.robot.subsystems.flywheel;

import frc.robot.util.RBSIIO;

public interface flywheelIO extends RBSIIO {

  public default void flywheelSetVelocity(double Velocity) {}

  public default void flywheelStop() {}

  public default double getVelocity(){
    return 0.0;
  }
}
