// Copyright 2026 FRC 2486
// https://github.com/Coconuts2486-FRC

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class VisionIOTest {
  private static final double EPSILON = 1e-9;

  @Test
  void limelightPoseParserUsesMetersAndDegrees() {
    double[] sample = {1.2, 2.3, 0.4, 10.0, -20.0, 90.0, 35.0, 2.0, 1.5, 3.0, 0.2};
    Pose3d pose = VisionIOLimelight.parsePose(sample);

    assertEquals(1.2, pose.getX(), EPSILON);
    assertEquals(2.3, pose.getY(), EPSILON);
    assertEquals(0.4, pose.getZ(), EPSILON);
    assertEquals(
        new Rotation3d(
            Units.degreesToRadians(10.0),
            Units.degreesToRadians(-20.0),
            Units.degreesToRadians(90.0)),
        pose.getRotation());
  }

  @Test
  void limelightPoseParserRejectsShortArrays() {
    assertThrows(IllegalArgumentException.class, () -> VisionIOLimelight.parsePose(new double[6]));
  }

  @Test
  void limelightSampleValidationRejectsMalformedSamples() {
    assertFalse(VisionIOLimelight.isValidBotPoseSample(new double[0]));
    double[] negativeTagCount = {0, 0, 0, 0, 0, 0, 20, -1, 0, 1, 0};
    assertFalse(VisionIOLimelight.isValidBotPoseSample(negativeTagCount));
    double[] valid = {0, 0, 0, 0, 0, 0, 20, 0, 0, 1, 0};
    assertTrue(VisionIOLimelight.isValidBotPoseSample(valid));
  }

  @Test
  void limelightUsedTagExtractionTrimsMissingPerTagData() {
    double[] full = {0, 0, 0, 0, 0, 0, 20, 2, 0, 1, 0, 7, 0, 0, 0, 0, 0, 0, 12, 0, 0, 0, 0, 0, 0};
    assertArrayEquals(new int[] {7, 12}, VisionIOLimelight.extractUsedTagIds(full));
    double[] truncated = {0, 0, 0, 0, 0, 0, 20, 2, 0, 1, 0, 7};
    assertArrayEquals(new int[] {7}, VisionIOLimelight.extractUsedTagIds(truncated));
  }

  @Test
  void limelightTimestampSubtractsLatencyMilliseconds() {
    double[] sample = {0, 0, 0, 0, 0, 0, 35, 1, 0, 1, 0};
    assertEquals(9.965, VisionIOLimelight.timestampSeconds(10_000_000L, sample), EPSILON);
  }

  @Test
  void poseObservationCoalescesNullUsedTagsToEmptyArray() {
    VisionIO.PoseObservation observation =
        new VisionIO.PoseObservation(
            1.0, new Pose3d(), 0.0, 1, 2.0, VisionIO.PoseObservationType.PHOTONVISION, null);
    assertArrayEquals(new int[0], observation.usedTagIds());
  }
}
