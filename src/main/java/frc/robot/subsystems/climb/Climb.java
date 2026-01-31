package frc.robot.subsystems.climb;

import frc.robot.util.RBSISubsystem;

public class Climb extends RBSISubsystem {
  private ClimbIO io;

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
