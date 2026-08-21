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
  private static final int BOTPOSE_MIN_LENGTH = 11;
  private static final int BOTPOSE_LATENCY_INDEX = 6;
  private static final int BOTPOSE_TAG_COUNT_INDEX = 7;
  private static final int BOTPOSE_AVG_DISTANCE_INDEX = 9;
  private static final int BOTPOSE_FIRST_TAG_ID_INDEX = 11;
  private static final int BOTPOSE_FIRST_TAG_AMBIGUITY_INDEX = 17;
  private static final int BOTPOSE_TAG_STRIDE = 7;
  private static final long ORIENTATION_FLUSH_PERIOD_US = 20_000;
  private static long lastOrientationFlushUs = 0;

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber heartbeatSubscriber;
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
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
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
        ((RobotController.getFPGATime() - heartbeatSubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    flushOrientationIfDue();

    // Read new pose observations from NetworkTables
    observedTagIds.clear();
    poseObservations.clear();

    var megatag1Samples = megatag1Subscriber.readQueue();
    for (int sampleIndex = megatag1Samples.length - 1; sampleIndex >= 0; sampleIndex--) {
      var sample = megatag1Samples[sampleIndex];
      if (addPoseObservation(sample.timestamp, sample.value, PoseObservationType.MEGATAG_1)) break;
    }

    var megatag2Samples = megatag2Subscriber.readQueue();
    for (int sampleIndex = megatag2Samples.length - 1; sampleIndex >= 0; sampleIndex--) {
      var sample = megatag2Samples[sampleIndex];
      if (addPoseObservation(sample.timestamp, sample.value, PoseObservationType.MEGATAG_2)) break;
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

  private boolean addPoseObservation(
      long timestampMicros, double[] rawPose, PoseObservationType observationType) {
    if (!isValidBotPoseSample(rawPose)) return false;

    int tagCount = getTagCount(rawPose);
    int[] usedTagIds = extractUsedTagIds(rawPose);
    for (int tagId : usedTagIds) observedTagIds.add(tagId);

    poseObservations.add(
        new PoseObservation(
            timestampSeconds(timestampMicros, rawPose),
            parsePose(rawPose),
            observationType == PoseObservationType.MEGATAG_1
                    && rawPose.length > BOTPOSE_FIRST_TAG_AMBIGUITY_INDEX
                ? rawPose[BOTPOSE_FIRST_TAG_AMBIGUITY_INDEX]
                : 0.0,
            tagCount,
            rawPose[BOTPOSE_AVG_DISTANCE_INDEX],
            observationType,
            usedTagIds));
    return true;
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  static Pose3d parsePose(double[] rawLLArray) {
    if (rawLLArray.length < BOTPOSE_MIN_LENGTH) {
      throw new IllegalArgumentException(
          "Limelight botpose array must have at least " + BOTPOSE_MIN_LENGTH + " values.");
    }
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  static boolean isValidBotPoseSample(double[] rawLLArray) {
    return rawLLArray.length >= BOTPOSE_MIN_LENGTH
        && getTagCount(rawLLArray) >= 0
        && Double.isFinite(rawLLArray[BOTPOSE_LATENCY_INDEX])
        && Double.isFinite(rawLLArray[BOTPOSE_AVG_DISTANCE_INDEX]);
  }

  static int[] extractUsedTagIds(double[] rawLLArray) {
    int tagCount = getTagCount(rawLLArray);
    int[] used = new int[Math.max(0, tagCount)];
    int count = 0;
    for (int i = BOTPOSE_FIRST_TAG_ID_INDEX;
        i < rawLLArray.length && count < used.length;
        i += BOTPOSE_TAG_STRIDE) {
      used[count++] = (int) rawLLArray[i];
    }
    return count == used.length ? used : Arrays.copyOf(used, count);
  }

  static double timestampSeconds(long ntTimestampMicros, double[] rawLLArray) {
    return ntTimestampMicros * 1.0e-6 - rawLLArray[BOTPOSE_LATENCY_INDEX] * 1.0e-3;
  }

  private static int getTagCount(double[] rawLLArray) {
    return (int) rawLLArray[BOTPOSE_TAG_COUNT_INDEX];
  }

  private static void flushOrientationIfDue() {
    long nowUs = RobotController.getFPGATime();
    if (nowUs - lastOrientationFlushUs < ORIENTATION_FLUSH_PERIOD_US) return;
    NetworkTableInstance.getDefault().flush();
    lastOrientationFlushUs = nowUs;
  }
}
