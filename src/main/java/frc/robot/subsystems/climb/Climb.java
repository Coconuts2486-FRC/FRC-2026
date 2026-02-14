package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.util.RBSISubsystem;

public class Climb extends RBSISubsystem {
  private ClimbIO io;

  private ElevatorFeedforward ffmodel;

  /** Constructor */
  public Climb(ClimbIO io) {
    // TODO Auto-generated constructor stub
    this.io = io;
  }

  @Override
  protected void rbsiPeriodic() {}

  public void setPosition(double pos) {
    io.setPosition(pos);
  }
}
