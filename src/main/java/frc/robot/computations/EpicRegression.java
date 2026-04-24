package frc.robot.computations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

public class EpicRegression {

  private static Pose2d shooter2d = Pose2d.kZero;
  private static Pose2d hub2d = Pose2d.kZero;
  private static Pose3d fieldLauncherPose;
  private static Translation2d translation;
  private static double v0;

  private EpicRegression() {}

  /** Regression Shot Solution Record */
  public record EpicShotSolution(double v0, Rotation2d psiField) {
    public double getVelocity() {
      return this.v0;
    }

    public Rotation2d getAngle() {
      return this.psiField;
    }
  }

  /**
   * @param fieldRobotPose robot/platform pose in FIELD frame
   * @param launcherTransformRobot transform from ROBOT origin to LAUNCHER exit (robot frame)
   * @param fieldTargetPose target pose in FIELD frame
   */
  public static EpicShotSolution solve(
      Pose3d fieldRobotPose,
      Transform3d launcherTransformRobot,
      Pose3d fieldTargetPose,
      Translation2d fieldPlatformVelocityMps) {
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
    v0 = BasicRegression.computeRegression(distance);

    // This is the robot YAW; compute field-relative angle
    double yaw = fieldLauncherPose.getRotation().getZ();
    double psiFieldRad = MathUtil.angleModulus(psi + yaw);

    // Shooting on the move!!!
    Rotation2d angleVRobot2HubVector =
        fieldPlatformVelocityMps.getAngle().minus(translation.getAngle());
    double vr = fieldPlatformVelocityMps.getNorm();

    // PLEASE NOTE: THESE SIGNS MAY BE WRONG!!!
    v0 +=
        vr
            * (Math.cos(angleVRobot2HubVector.getRadians()))
            / Math.cos(Units.degreesToRadians(65))
            / ShooterConstants.flywheelCircumfrence
            * 60;

    psiFieldRad +=
        vr
            * (Math.sin(angleVRobot2HubVector.getRadians()))
            * ShooterConstants.timeOfFlight
            / distance;

    if (Math.sqrt(
            Math.pow(Math.abs((fieldRobotPose.getY() - fieldTargetPose.getY())), 2)
                + Math.pow(Math.abs((fieldRobotPose.getX() - fieldTargetPose.getX())), 2))
        > 1.5) {
      return new EpicShotSolution(v0, Rotation2d.fromRadians(psiFieldRad));
    } else {
      return new EpicShotSolution(v0 + 0.05, Rotation2d.fromRadians(psiFieldRad));
    }
  }
}
