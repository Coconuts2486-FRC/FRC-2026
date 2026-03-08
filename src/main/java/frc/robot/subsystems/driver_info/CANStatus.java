package frc.robot.subsystems.driver_info;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class CANStatus extends VirtualSubsystem {

  private final Drive drive;
  private final Imu imu;

  private final Intake intake;
  private final Feeder feeder;

  Boolean driveCAN = false;
  Boolean mainCAN = false;

  /** Constructor */
  public CANStatus(Drive drive, Imu imu, Intake intake, Feeder feeder) {
    this.drive = drive;
    this.imu = imu;
    this.intake = intake;
    this.feeder = feeder;
  }

  @Override
  public void rbsiPeriodic() {

    // figure out if the drive CAN network is alive
    var modules = drive.getModules();

    driveCAN =
        (modules[0].isAlive()
            && modules[1].isAlive()
            && modules[2].isAlive()
            && modules[3].isAlive()
            && imu.isConnected());

    mainCAN = (intake.pivotAlive() && intake.rollersAlive() && feeder.isFeederAlive());

    // Logger inputs for each CAN network
    Logger.recordOutput("CAN/DriveCAN", driveCAN);
    Logger.recordOutput("CAN/MainCAN", mainCAN);

    // Logger inputs for each part of the drive CAN network
    Logger.recordOutput("CAN/PigeonALive", imu.isConnected());
    Logger.recordOutput("CAN/Module1Alive", modules[0].isAlive());
    Logger.recordOutput("CAN/Module2Alive", modules[1].isAlive());
    Logger.recordOutput("CAN/Module3Alive", modules[2].isAlive());
    Logger.recordOutput("CAN/Module4Alive", modules[3].isAlive());

    // logger inputs for each part of the main CAN network
    Logger.recordOutput("CAN/IntakePivotAlive", intake.pivotAlive());
    Logger.recordOutput("CAN/IntakeRollersAlive", intake.rollersAlive());
    Logger.recordOutput("CAN/FeederAlive", feeder.isFeederAlive());
  }
}
