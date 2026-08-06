// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final Set<Integer> observedTagIds = new HashSet<>();
  private final List<PoseObservation> poseObservations = new ArrayList<>();

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    observedTagIds.clear();
    poseObservations.clear();

    var megatag1Samples = megatag1Subscriber.readQueue();
    if (megatag1Samples.length > 0) {
      var newest = megatag1Samples[megatag1Samples.length - 1];
      addPoseObservation(newest.timestamp, newest.value, PoseObservationType.MEGATAG_1);
    }

    var megatag2Samples = megatag2Subscriber.readQueue();
    if (megatag2Samples.length > 0) {
      var newest = megatag2Samples[megatag2Samples.length - 1];
      addPoseObservation(newest.timestamp, newest.value, PoseObservationType.MEGATAG_2);
    }

    // Save pose observations to inputs object
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[observedTagIds.size()];
    int i = 0;
    for (int id : observedTagIds) {
      inputs.tagIds[i++] = id;
    }

    // Sort list by TagID for clarity
    Arrays.sort(inputs.tagIds);
  }

  private void addPoseObservation(
      long timestampMicros, double[] rawPose, PoseObservationType observationType) {
    if (rawPose.length < 10) {
      return;
    }

    int tagCount = Math.max(0, (int) rawPose[7]);
    int availableTagCount = Math.max(0, (rawPose.length - 5) / 7);
    int[] usedTagIds = new int[Math.min(tagCount, availableTagCount)];
    int usedTagCount = 0;
    for (int i = 11; i < rawPose.length && usedTagCount < usedTagIds.length; i += 7) {
      int tagId = (int) rawPose[i];
      usedTagIds[usedTagCount++] = tagId;
      observedTagIds.add(tagId);
    }

    poseObservations.add(
        new PoseObservation(
            timestampMicros * 1.0e-6 - rawPose[6] * 1.0e-3,
            parsePose(rawPose),
            observationType == PoseObservationType.MEGATAG_1 && rawPose.length >= 18
                ? rawPose[17]
                : 0.0,
            tagCount,
            rawPose[9],
            observationType,
            usedTagIds));
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
