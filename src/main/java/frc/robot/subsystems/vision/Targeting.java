package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import java.util.Optional;

/**
 * Gameplay-only vision helper.
 *
 * <p>Responsibilities: - Consume "target observations" (tx/ty + timestamp + tag id if available) -
 * Choose which target/tag is relevant for the current objective - Compute desired robot heading /
 * aiming solution - Provide "isAimed"/"isTargetStable" signals
 *
 * <p>Non-responsibilities: - Pose estimator injection - Multi-camera pose fusion - Vision std-dev /
 * trust model for odometry
 */
public class Targeting {
  /** Which goal are we trying to aim at right now? */
  public enum GoalMode {
    NONE,
    REDHUB,
    BLUEHUB,
    SOURCE,
    CUSTOM_TAG
  }

  /** A minimal input per camera (you can enrich this later). */
  public record CameraTargetSample(
      int cameraIndex,
      double timestampSeconds, // camera capture time, in FPGA timebase if possible
      Rotation2d tx,
      Rotation2d ty,
      boolean hasTarget,
      int bestTagId // -1 if unknown
      ) {}

  /** The chosen target solution (output of targeting). */
  public record TargetSolution(
      double timestampSeconds, // the time this solution corresponds to
      int tagId, // tag used (or -1)
      Rotation2d desiredHeading, // field-relative robot heading to face the target
      double confidence, // 0..1
      Rotation2d tx, // observed tx (optional)
      Rotation2d ty // observed ty (optional)
      ) {}

  /** External dependencies (usually passed from Coordinator/RobotContainer). */
  public interface PoseSampler {
    /** Current pose. */
    Pose2d getPose();

    /** Pose at time (optional). */
    Optional<Pose2d> getPoseAtTime(double timestampSeconds);
  }

  // ------------------------------ Tunables ------------------------------

  // Aim tolerances
  private static final double kAimToleranceRad = Math.toRadians(2.0);
  private static final double kStableWindowSec = 0.20;

  // Basic yaw controller for aiming (replace with your own PID if you already have one)
  private static final double kAimKp = 4.0; // rad/s per rad error (example)

  // ------------------------------ State ------------------------------

  private final PoseSampler poseSampler;

  private GoalMode goalMode = GoalMode.NONE;
  private int customTagId = -1;

  // Most recent per-camera target samples (simple: store "latest"; can become ring buffers later)
  private CameraTargetSample[] latestByCamera = new CameraTargetSample[0];

  // Last chosen solution
  private Optional<TargetSolution> lastSolution = Optional.empty();

  // Stability tracking
  private double lastGoodAimTimestamp = Double.NEGATIVE_INFINITY;

  public Targeting(PoseSampler poseSampler, int cameraCount) {
    this.poseSampler = poseSampler;
    this.latestByCamera = new CameraTargetSample[cameraCount];
  }

  // ------------------------------ Configuration ------------------------------

  public void setGoalMode(GoalMode mode) {
    this.goalMode = mode;
  }

  public void setCustomTagId(int tagId) {
    this.customTagId = tagId;
    this.goalMode = GoalMode.CUSTOM_TAG;
  }

  // ------------------------------ Input update ------------------------------

  /**
   * Feed one camera’s latest target observation into targeting (called from VisionIO layer or a
   * wrapper).
   */
  public void updateCameraSample(CameraTargetSample sample) {
    if (sample.cameraIndex < 0 || sample.cameraIndex >= latestByCamera.length) return;

    var prev = latestByCamera[sample.cameraIndex];
    // Keep newest by timestamp so stale frames don't overwrite
    if (prev == null || sample.timestampSeconds >= prev.timestampSeconds) {
      latestByCamera[sample.cameraIndex] = sample;
    }
  }

  // ------------------------------ Public outputs ------------------------------

  /** Returns the best current target solution for the active goal mode. */
  public Optional<TargetSolution> getBestTarget() {
    return lastSolution;
  }

  /**
   * True if we have a solution and the heading error is within tolerance for a short stable window.
   */
  public boolean isAimed() {
    if (lastSolution.isEmpty()) return false;

    Pose2d pose = poseSampler.getPose();
    double err =
        MathUtil.angleModulus(
            lastSolution.get().desiredHeading.getRadians() - pose.getRotation().getRadians());

    boolean within = Math.abs(err) <= kAimToleranceRad;
    double now = Timer.getFPGATimestamp();

    if (within) {
      if (now - lastGoodAimTimestamp > 0.02) {
        // refresh; this is a simple latch to measure stability
        lastGoodAimTimestamp = now;
      }
      // Require the condition to have been true recently (stability)
      return (now - lastGoodAimTimestamp) <= kStableWindowSec;
    } else {
      // reset
      lastGoodAimTimestamp = Double.NEGATIVE_INFINITY;
      return false;
    }
  }

  /** Convenience: compute desired robot heading from most recent solution. */
  public double getDesiredRobotHeadingRad(Pose2d robotPose, TargetSolution sol) {
    return sol.desiredHeading.getRadians();
  }

  /**
   * Build a drive request that turns toward desiredHeading. Keep translation from driver, or return
   * just omega.
   */
  public ChassisSpeeds buildAimingDriveRequest(double desiredHeadingRad) {
    Pose2d pose = poseSampler.getPose();
    double err = MathUtil.angleModulus(desiredHeadingRad - pose.getRotation().getRadians());
    double omega = MathUtil.clamp(kAimKp * err, -6.0, 6.0); // clamp example
    return new ChassisSpeeds(0.0, 0.0, omega);
  }

  // ------------------------------ Main periodic solver ------------------------------

  /**
   * Call this once per loop from Coordinator (or from Targeting itself if it becomes a subsystem).
   */
public void periodic() {
    Optional<CameraTargetSample> bestSample = chooseBestSampleForGoal();

    if (bestSample.isPresent() && bestSample.get().hasTarget) {
        CameraTargetSample s = bestSample.get();
        Pose2d poseAtSample =
            poseSampler.getPoseAtTime(s.timestampSeconds).orElseGet(poseSampler::getPose);
        Rotation2d desired = computeDesiredHeading(poseAtSample, s);
        double confidence = s.bestTagId >= 0 ? 0.9 : 0.7;
        lastSolution = Optional.of(
            new TargetSolution(s.timestampSeconds, s.bestTagId, desired, confidence, s.tx, s.ty));
        return;
    }

    // No tag visible — fall back to odometry
    Rotation2d odometryHeading = computeOdometryHeading();
    if (odometryHeading != null) {
        lastSolution = Optional.of(
            new TargetSolution(
                Timer.getFPGATimestamp(),
                -1,
                odometryHeading,
                0.8, 
                Rotation2d.kZero,
                Rotation2d.kZero));
    } else {
        lastSolution = Optional.empty();
    }
}

private Rotation2d computeOdometryHeading() {
    // Get the known hub position based on current goal mode
    edu.wpi.first.math.geometry.Translation2d hubTarget;
    if (goalMode == GoalMode.REDHUB) {
        hubTarget = new edu.wpi.first.math.geometry.Translation2d(
            frc.robot.FieldConstants.hubCenterRed.getX(),
            frc.robot.FieldConstants.hubCenterRed.getY());
    } else if (goalMode == GoalMode.BLUEHUB) {
        hubTarget = new edu.wpi.first.math.geometry.Translation2d(
            frc.robot.FieldConstants.hubCenterBlue.getX(),
            frc.robot.FieldConstants.hubCenterBlue.getY());
    } else {
        return null; // No known target for other modes
    }

    Pose2d pose = poseSampler.getPose();
    return hubTarget.minus(pose.getTranslation()).getAngle();
}

  // ------------------------------ Internals ------------------------------

  private Optional<CameraTargetSample> chooseBestSampleForGoal() {
    CameraTargetSample best = null;

    for (var s : latestByCamera) {
      if (s == null || !s.hasTarget) continue;

      if (goalMode == GoalMode.REDHUB) {
        boolean isREDHUB = false;

        for (int id : FieldConstants.REDHUB_TAG_IDS) {
          if (s.bestTagId == id) {
            isREDHUB = true;
            break;
          }
        }

        if (!isREDHUB) continue;
      }

      if (goalMode == GoalMode.BLUEHUB) {
        boolean isBLUEHUB = false;
        for (int id : FieldConstants.BLUEHUB_TAG_IDS) {
          if (s.bestTagId == id) {
            isBLUEHUB = true;
            break;
          }
        }
        if (!isBLUEHUB) continue;
      }

      // If we’re in CUSTOM_TAG mode, require that tag (if we have tag IDs)
      if (goalMode == GoalMode.CUSTOM_TAG && customTagId >= 0) {
        if (s.bestTagId != customTagId) continue;
      }

      // Pick newest sample for now (you can improve: pick highest confidence, smallest |tx|, etc.)
      if (best == null || s.timestampSeconds > best.timestampSeconds) {
        best = s;
      }
    }

    return Optional.ofNullable(best);
  }

  private Rotation2d computeDesiredHeading(Pose2d poseAtSample, CameraTargetSample s) {
    // Option A (simple): If tx is "yaw error" in robot frame, desiredHeading = currentHeading + tx
    // IMPORTANT: sign conventions vary; you might need -tx.
    double desiredRad = poseAtSample.getRotation().getRadians() + s.tx.getRadians();
    return Rotation2d.fromRadians(desiredRad);

    // Option B (better): Use known field position of target/tag -> compute vector to it -> atan2
    // That requires FieldConstants + tag pose lookup, and potentially using a range estimate.
  }
}
