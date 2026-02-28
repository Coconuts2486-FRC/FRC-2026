package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO extends RBSIIO {

  @AutoLog
  public static class ClimbIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setPosition(double pos) {}

  public default double getPosition() {
    return 0.0;
  }

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
