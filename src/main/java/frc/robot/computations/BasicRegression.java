package frc.robot.computations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class BasicRegression {

  private static Pose2d shooter2d = Pose2d.kZero;
  private static Pose2d hub2d = Pose2d.kZero;
  private static Pose3d fieldLauncherPose;
  private static Translation2d translation;
  private static double v0;

  private BasicRegression() {}

  /** Regression Shot Solution Record */
  public record RegressionShotSolution(double v0, Rotation2d psiField) {
    public double getVelocity() {
      return this.v0;
    }

    public Rotation2d getAngle() {
      return this.psiField;
    }
  }

  /**
   * Compute the regression for v0 given the distance in meters
   *
   * @param distance Distance of the shot in meters
   */
  private static double computeRegression(double distance) {
    double A = -25.55419; // Constant
    double B = 37.01029; // Linear in distance
    double C = -12.04535; // Quadratic in distance
    double D = 1.39996; // Cubic in distance
    // double E = 0.0; // Quartic in distance

    return A + B * distance + C * distance * distance + D * distance * distance * distance;
    //      + E * distance * distance * distance * distance;
  }

  /**
   * @param fieldRobotPose robot/platform pose in FIELD frame
   * @param launcherTransformRobot transform from ROBOT origin to LAUNCHER exit (robot frame)
   * @param fieldTargetPose target pose in FIELD frame
   */
  public static RegressionShotSolution solve(
      Pose3d fieldRobotPose, Transform3d launcherTransformRobot, Pose3d fieldTargetPose) {
    // Launcher pose in field frame
    fieldLauncherPose = fieldRobotPose.plus(launcherTransformRobot);

    // // Target expressed in launcher frame
    // targetInLauncherFrame = new Transform3d(fieldLauncherPose, fieldTargetPose);

    // Distance to target
    shooter2d = fieldLauncherPose.toPose2d();
    hub2d = fieldTargetPose.toPose2d();
    translation = hub2d.relativeTo(shooter2d).getTranslation();
    double distance = translation.getNorm();
    double psi = translation.getAngle().getRadians();

    // Compute the velocity from the regression
    v0 = computeRegression(distance);

    // This is the robot YAW; compute field-relative angle
    double yaw = fieldLauncherPose.getRotation().getZ();
    double psiFieldRad = MathUtil.angleModulus(psi + yaw);

    return new RegressionShotSolution(v0, Rotation2d.fromRadians(psiFieldRad));
  }
}
