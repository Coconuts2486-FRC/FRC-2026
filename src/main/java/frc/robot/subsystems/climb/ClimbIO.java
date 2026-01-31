package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.RBSIIO;

public interface ClimbIO extends RBSIIO {

  public default void extendToPos(double pos) {}

  public default void configure(
      double Kg,
      double Ks,
      double Kv,
      double Ka,
      double Kp,
      double Ki,
      double Kd,
      LinearVelocity velocity,
      LinearAcceleration aceleration,
      double jerk) {}
}
