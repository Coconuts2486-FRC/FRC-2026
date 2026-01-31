package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.util.RBSISubsystem;

public class Climb extends RBSISubsystem {
  private ClimbIO io;

  private ElevatorFeedforward ffmodel;

  public Climb(ClimbIOTalonFX climbIOTalonFX) {
    // TODO Auto-generated constructor stub
  }

  public void Climb(ClimbIO io) {
    this.io = io;
  }

  public void extendToPos(double pos) {
    io.extendToPos(pos);
  }
}
