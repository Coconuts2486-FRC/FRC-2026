package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.RBSISubsystem;

public class rollers extends RBSISubsystem {
  private rollersIO io;

  public rollers(rollersIO io) {
    this.io = io;

    setDefaultCommand(Commands.run(() -> stop(), this));
  }

  @Override
  public void rbsiPeriodic() {}

  public void runRollers() {
    io.runRollers(0.5);
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
