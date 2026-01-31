package frc.robot.subsystems.climb;

import frc.robot.util.RBSIIO;

public interface ClimbIO extends RBSIIO {

  public default void extendToPos(double pos) {}
}
