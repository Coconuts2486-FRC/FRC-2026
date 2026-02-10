package frc.robot.subsystems.Driver_Info;

import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.imu.ImuIO;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class CANStatus extends VirtualSubsystem {

  Boolean driveCAN = false;
  Boolean mainCAN = false;

  public void can_status() {}

  @Override
  public void rbsiPeriodic() {

    // var testStatus =
    //     BaseStatusSignal.refreshAll(
    //         test.getStatorCurrent(),
    //         test.getPosition(),
    //         test.getVelocity(),
    //         test.getMotorVoltage());
    // System.out.println(testStatus.isOK());
    // example code for checking if motor is good

    // if () {
    //   mainCAN = true;
    // } else {
    //   mainCAN = false;
    // }

    // if statement to figure out if the drive CAN network is alive
    if (Module.alive1
        && Module.alive2
        && Module.alive3
        && Module.alive4
        && ImuIO.ImuIOInputs.connected) {
      driveCAN = true;
    } else {
      driveCAN = false;
    }

    // Logger inputs for each CAN network
    Logger.recordOutput("CAN/DriveCAN", driveCAN);
    Logger.recordOutput("CAN/MainCAN", mainCAN);

    // Logger inputs for each part of the drive CAN network
    Logger.recordOutput("CAN/PigeonALive", ImuIO.ImuIOInputs.connected);
    Logger.recordOutput("CAN/Module1Alive", Module.alive1);
    Logger.recordOutput("CAN/Module2Alive", Module.alive2);
    Logger.recordOutput("CAN/Module3Alive", Module.alive3);
    Logger.recordOutput("CAN/Module4Alive", Module.alive4);

    // logger inputs for each part of the main CAN network
    // Logger.recordOutput("CAN/TestAlive", testStatus.isOK());
  }
}
