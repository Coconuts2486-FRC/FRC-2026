package frc.robot.computations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class BasicRegression {

  private static Pose2d shooter2d = Pose2d.kZero;
  private static Pose2d hub2d = Pose2d.kZero;

  private BasicRegression() {}

  public static double distanceToHub(Pose3d shooter, Pose3d hub) {
    shooter2d = shooter.toPose2d();
    hub2d = hub.toPose2d();

    return shooter2d.relativeTo(hub2d).getTranslation().getNorm();
  }
  ;
}
