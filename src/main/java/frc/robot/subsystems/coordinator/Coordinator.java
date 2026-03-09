package frc.robot.subsystems.coordinator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.computations.FieldRelativeShooterSolver.FieldShotSolution;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;

public class Coordinator extends VirtualSubsystem {
  public enum Mode {
    SYSTEM_CHECK, // Disabled, pre-match
    IDLE, // Disabled, playing defense, climb
    INTAKE, // Filling the hopper
    SCORE, // Shooting FUEL to the HUB
    PASS, // Shooting FUEL to our alliance zone
    CLIMB // Climbing
  }

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Translation2d> velocitySupplier;
  private final Supplier<Boolean> intakeRollersRunningSupplier;
  private final Supplier<Boolean> intakeExtendedSupplier;
  private Mode mode = Mode.IDLE;

  // latched "intent" flags (set by commands/buttons)
  private boolean wantAutoAim = false;
  private boolean wantScore = false;

  // Internal variables
  private static boolean ok_to_shoot = false;
  public static Pose3d target = null;
  private static FieldShotSolution fuelSolution;

  private enum Zones {
    HOME_ZONE,
    NEUTRAL_ZONE,
    FOREIGN_ZONE
  }

  private Zones zone;

  /** Constructor */
  public Coordinator(
      Supplier<Pose2d> poseSupplier,
      Supplier<Translation2d> velocitySupplier,
      Supplier<Boolean> intakeRollersRunningSupplier,
      Supplier<Boolean> intakeExtendedSupplier) {
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
    this.intakeRollersRunningSupplier = intakeRollersRunningSupplier;
    this.intakeExtendedSupplier = intakeExtendedSupplier;
  }

  public void setMode(Mode mode) {
    this.mode = mode;
  }

  public void setWantAutoAim(boolean enabled) {
    wantAutoAim = enabled;
  }

  public void requestScore() {
    wantScore = true;
  }

  @Override
  public void rbsiPeriodic() {
    if (DriverStation.isDisabled()) {
      // Always safe outputs
      return;
    }

    // Read in the current robot state “truth”
    Pose2d pose = poseSupplier.get();
    double xpos = pose.getX();
    double ypos = pose.getY();
    Translation2d velocity = velocitySupplier.get();
    Alliance alliance = DriverStation.getAlliance().get();

    // Determine whether we are in the HOME, NEUTRAL, or FOREIGN zone
    if (xpos < FieldConstants.startingLineXBLue.in(Meters)) {
      // On the BLUE side of the field
      zone =
          switch (alliance) {
            case Blue -> Zones.HOME_ZONE;
            case Red -> Zones.FOREIGN_ZONE;
          };

    } else if (xpos > FieldConstants.startingLineXRed.in(Meters)) {
      // On the RED side of the field
      zone =
          switch (alliance) {
            case Red -> Zones.HOME_ZONE;
            case Blue -> Zones.FOREIGN_ZONE;
          };
    } else {
      // In the NEUTRAL ZONE
      zone = Zones.NEUTRAL_ZONE;
    }

    // Choose shooting action based on zone
    switch (zone) {
      case HOME_ZONE:
        // Aim turret at our hub
        target =
            switch (alliance) {
              case Blue -> FieldConstants.hubCenterBlue;
              case Red -> FieldConstants.hubCenterRed;
            };

        break;

      case NEUTRAL_ZONE:
        // Aim turret at one of two passing locations based on Y position
        switch (alliance) {
          case Blue:
            target =
                (ypos < FieldConstants.aprilTagLayout.getFieldWidth() / 2.)
                    ? FieldConstants.passingOutpostBlue
                    : FieldConstants.passingDepotBlue;

          case Red:
            target =
                (ypos > FieldConstants.aprilTagLayout.getFieldWidth() / 2.)
                    ? FieldConstants.passingOutpostRed
                    : FieldConstants.passingDepotRed;
        }
        break;

      case FOREIGN_ZONE:
        // Do nothing!
        target = Pose3d.kZero;
        break;
    }

    // Using the target and the current pose, compute v0 and phi
    // fuelSolution =
    //     FieldRelativeShooterSolver.solve(new Pose3d(pose), Transform3d.kZero, target, velocity);

    // Check on intake roller running
    boolean intakeRunning = intakeRollersRunningSupplier.get();

    // 2) State machine / mode logic
    switch (mode) {
      case IDLE -> {
        // default behavior (maybe driver control only)
      }

      // case AIM -> {
      //   if (wantAutoAim && tgt.isPresent()) {
      //     // Example: compute desired heading from target solution
      //     double desiredHeadingRad = targeting.getDesiredRobotHeadingRad(pose, tgt.get());

      //     // Produce a chassis request (you might have your own helper)
      //     ChassisSpeeds speeds = targeting.buildAimingDriveRequest(desiredHeadingRad);
      //     drive.runVelocity(speeds);
      //   }
      // }

      case SCORE -> {
        // Example: require "aimed + shooter ready" then feed
        boolean ready = true; // shooter.atSetpoint(), etc

        if (ready && wantScore) {
          // feed
          wantScore = false;
        }
      }

      case INTAKE -> {
        // intake logic
      }

      case CLIMB -> {
        // climb logic
      }

      default -> {}
    }

    // 3) Log coordinator outputs for tuning
    // Logger.recordOutput("Coord/Mode", mode.toString());
    // Logger.recordOutput("Coord/WantAutoAim", wantAutoAim);

    // But really do this based on calculating things!
    // Based on field position, HUB active, turret in position, flywheel at speed, override not set,
    // "DON'T SHOOT" button not pressed...
    ok_to_shoot = true;
  }

  // Getter functions
  public static boolean getOk2Shoot() {
    return ok_to_shoot;
  }

  public static double getShooterVelocity() {
    return fuelSolution.getVelocity();
  }

  public static Rotation2d getTurretAngle() {
    return fuelSolution.getAngle();
  }
}
