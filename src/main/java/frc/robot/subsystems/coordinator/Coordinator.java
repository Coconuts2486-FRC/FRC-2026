package frc.robot.subsystems.coordinator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Targeting;
import frc.robot.util.VirtualSubsystem;

public class Coordinator extends VirtualSubsystem {
  public enum Mode {
    IDLE,
    INTAKE,
    AIM,
    SCORE,
    CLIMB
  }

  private final Drive drive;
  private final Targeting targeting;
  // add shooter/intake/etc here as needed

  private Mode mode = Mode.IDLE;

  // latched “intent” flags (set by commands/buttons)
  private boolean wantAutoAim = false;
  private boolean wantScore = false;

  // Internal variables
  private static boolean ok_to_shoot = false;

  public Coordinator(Drive drive, Targeting targeting /*, Shooter shooter, Intake intake */) {
    this.drive = drive;
    this.targeting = targeting;
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
      wantScore = false;
      // stop mechanisms etc
      return;
    }

    // 1) Read “truth”
    Pose2d pose = drive.getPose();
    var tgt = targeting.getBestTarget(); // Optional / nullable record

    // 2) State machine / mode logic
    switch (mode) {
      case IDLE -> {
        // default behavior (maybe driver control only)
      }

      case AIM -> {
        if (wantAutoAim && tgt.isPresent()) {
          // Example: compute desired heading from target solution
          double desiredHeadingRad = targeting.getDesiredRobotHeadingRad(pose, tgt.get());

          // Produce a chassis request (you might have your own helper)
          ChassisSpeeds speeds = targeting.buildAimingDriveRequest(desiredHeadingRad);
          drive.runVelocity(speeds);
        }
      }

      case SCORE -> {
        // Example: require "aimed + shooter ready" then feed
        boolean aimed = targeting.isAimed();
        boolean ready = true; // shooter.atSetpoint(), etc

        if (aimed && ready && wantScore) {
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
}
