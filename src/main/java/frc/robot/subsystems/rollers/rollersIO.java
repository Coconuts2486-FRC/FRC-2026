package frc.robot.subsystems.rollers;

import frc.robot.util.RBSIIO;

public interface rollersIO extends RBSIIO {

  public default boolean isIntakeRollersRunning() {
    return false;
  }

  public default void runRollers(double speed) {}

  public default void stop() {}
}
