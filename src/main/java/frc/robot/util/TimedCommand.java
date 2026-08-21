// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First

package frc.robot.util;

import org.wpilib.command2.Command;
import org.wpilib.command2.WrapperCommand;
import org.littletonrobotics.junction.Logger;

/** Logs each lifecycle phase of a wrapped command without changing its scheduling semantics. */
public final class TimedCommand extends WrapperCommand {
  private final String logKey;

  public TimedCommand(Command command, String logKey) {
    super(command);
    this.logKey = logKey;
  }

  @Override
  public void initialize() {
    final long start = System.nanoTime();
    super.initialize();
    Logger.recordOutput(logKey + "/InitializeMS", (System.nanoTime() - start) / 1e6);
  }

  @Override
  public void execute() {
    final long start = System.nanoTime();
    super.execute();
    Logger.recordOutput(logKey + "/ExecuteMS", (System.nanoTime() - start) / 1e6);
  }

  @Override
  public void end(boolean interrupted) {
    final long start = System.nanoTime();
    super.end(interrupted);
    Logger.recordOutput(logKey + "/EndMS", (System.nanoTime() - start) / 1e6);
  }
}
