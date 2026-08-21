// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.FieldConstants.*;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  private static final PoseObservation[] kNoPoseObservations = new PoseObservation[0];
  private static final int[] kNoTagIds = new int[0];

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Set<Integer> unionTagIds = new HashSet<>();
  private final PoseObservation[] poseObservationOutput = new PoseObservation[1];
  private int[] tagIdOutput = kNoTagIds;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    unionTagIds.clear();
    poseObservationOutput[0] = null;

    if (!inputs.connected) {
      inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      inputs.poseObservations = kNoPoseObservations;
      inputs.tagIds = kNoTagIds;
      return;
    }

    double newestTargetTs = Double.NEGATIVE_INFINITY;
    Rotation2d bestYaw = Rotation2d.kZero;
    Rotation2d bestPitch = Rotation2d.kZero;

    final var unreadResults = camera.getAllUnreadResults();
    if (!unreadResults.isEmpty()) {
      // PhotonVision returns unread changes in FIFO order. Drain the FIFO, but retain only the
      // newest result so a slow robot loop cannot create a camera-processing backlog.
      var result = unreadResults.get(unreadResults.size() - 1);

      final double ts = result.getTimestampSeconds();

      if (result.hasTargets() && ts >= newestTargetTs) {
        newestTargetTs = ts;
        bestYaw = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
        bestPitch = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitag = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitag.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }
        double avgTagDistance =
            result.targets.isEmpty() ? 0.0 : (totalTagDistance / result.targets.size());

        // Build used tag list (loggable + replayable)
        int[] used = new int[multitag.fiducialIDsUsed.size()];
        int u = 0;
        for (int id : multitag.fiducialIDsUsed) {
          used[u++] = id;
          unionTagIds.add(id); // keep your union set for tagIds UI/log
        }

        // Add observation
        poseObservationOutput[0] =
            new PoseObservation(
                ts,
                robotPose,
                multitag.estimatedPose.ambiguity,
                multitag.fiducialIDsUsed.size(),
                avgTagDistance,
                PoseObservationType.PHOTONVISION,
                used);

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;

          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          unionTagIds.add(target.fiducialId);

          poseObservationOutput[0] =
              new PoseObservation(
                  ts,
                  robotPose,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm(),
                  PoseObservationType.PHOTONVISION,
                  new int[] {target.fiducialId});
        }
      }
    }

    // Save pose observations to inputs object
    inputs.latestTargetObservation =
        (newestTargetTs > Double.NEGATIVE_INFINITY)
            ? new TargetObservation(bestYaw, bestPitch)
            : new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    inputs.poseObservations =
        poseObservationOutput[0] == null ? kNoPoseObservations : poseObservationOutput;

    // Save tag IDs to inputs objects
    if (unionTagIds.isEmpty()) {
      inputs.tagIds = kNoTagIds;
      return;
    }
    if (tagIdOutput.length != unionTagIds.size()) {
      tagIdOutput = new int[unionTagIds.size()];
    }
    inputs.tagIds = tagIdOutput;
    int i = 0;
    for (int id : unionTagIds) inputs.tagIds[i++] = id;

    // Sort the AprilTag IDs for ease of use by dashboards, etc.
    Arrays.sort(inputs.tagIds);
  }
}
