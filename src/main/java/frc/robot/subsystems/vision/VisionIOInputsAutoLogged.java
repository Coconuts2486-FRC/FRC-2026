package frc.robot.subsystems.vision;

/** Compatibility name retained after switching vision inputs to explicit serialization. */
public final class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements Cloneable {
  @Override
  public VisionIOInputsAutoLogged clone() {
    final VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.connected = connected;
    copy.latestTargetObservation = latestTargetObservation;
    copy.poseObservations = poseObservations.clone();
    copy.tagIds = tagIds.clone();
    return copy;
  }
}
