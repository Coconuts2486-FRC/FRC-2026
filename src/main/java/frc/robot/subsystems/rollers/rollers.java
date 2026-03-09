package frc.robot.subsystems.rollers;

import frc.robot.util.RBSISubsystem;

public class rollers extends RBSISubsystem {
  private rollersIO io;

  public rollers(rollersIO io) {
    this.io = io;
  }

  @Override
  public void rbsiPeriodic() {}

  public void runRollers(double speed) {
    io.runRollers(speed);
  }

  public void stop() {
    io.stop();
  }

  public boolean isIntakeRollersRunning() {
    return io.isIntakeRollersRunning();
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
