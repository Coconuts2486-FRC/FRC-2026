package frc.robot.subsystems.Driver_Info;

import frc.robot.subsystems.drive.Module;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class can_status extends VirtualSubsystem {

  Boolean driveCAN = false;

  public void can_status() {}

  @Override
  public void periodic() {
    if (Module.alive1 && Module.alive2 && Module.alive3 && Module.alive4) {
      driveCAN = true;
    } else {
      driveCAN = false;
    }

    Logger.recordOutput("CAN/DriveCAN", driveCAN);
  }
}
