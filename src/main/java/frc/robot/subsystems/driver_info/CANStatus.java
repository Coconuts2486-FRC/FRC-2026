package frc.robot.subsystems.driver_info;

import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class CANStatus extends VirtualSubsystem {

  private final Drive drive;
  private final Imu imu;
  private final Climb climb;

  private final Intake intake;
  private final Feeder feeder;
  private final rollers rollers;
  private final Shooter shooter;
  private final Turret turret;
  private final Indexer indexer;

  Boolean driveCAN = false;
  Boolean mainCAN = false;

  /** Constructor */
  public CANStatus(
      Drive drive,
      Imu imu,
      Intake intake,
      Feeder feeder,
      rollers rollers,
      Shooter shooter,
      Turret turret,
      Indexer indexer,
      Climb climb) {
    this.drive = drive;
    this.imu = imu;
    this.climb = climb;

    this.intake = intake;
    this.feeder = feeder;
    this.rollers = rollers;
    this.shooter = shooter;
    this.turret = turret;
    this.indexer = indexer;
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
            && imu.isConnected()
            && climb.climbAlive());

    mainCAN =
        (intake.pivotAlive()
            && feeder.isFeederAlive()
            && Math.abs(intake.getPivotPosition()) > 0.0
            && rollers.isRollersAlive()
            && shooter.leaderAlive()
            && shooter.followerAlive()
            && turret.turretAlive()
            && Math.abs(turret.getTurretEncoderPosition()) > 0.0
            && indexer.indexerAlive());

    // Logger inputs for each CAN network
    Logger.recordOutput("CAN/DriveCAN", driveCAN);
    Logger.recordOutput("CAN/MainCAN", mainCAN);

    // Logger inputs for each part of the drive CAN network
    Logger.recordOutput("CAN/PigeonALive", imu.isConnected());
    Logger.recordOutput("CAN/Module1Alive", modules[0].isAlive());
    Logger.recordOutput("CAN/Module2Alive", modules[1].isAlive());
    Logger.recordOutput("CAN/Module3Alive", modules[2].isAlive());
    Logger.recordOutput("CAN/Module4Alive", modules[3].isAlive());
    Logger.recordOutput("CAN/ClimbALive", climb.climbAlive());

    // logger inputs for each part of the main CAN network
    Logger.recordOutput("CAN/IntakePivotAlive", intake.pivotAlive());
    Logger.recordOutput("CAN/FeederAlive", feeder.isFeederAlive());
    Logger.recordOutput("CAN/IntakeCancoderAlive", Math.abs(intake.getPivotPosition()) > 0.0);
    Logger.recordOutput("CAN/IntakeRollersAlive", rollers.isRollersAlive());
    Logger.recordOutput("CAN/FlywheelLeaderAlive", shooter.leaderAlive());
    Logger.recordOutput("CAN/FlywheelFollowerAlive", shooter.followerAlive());
    Logger.recordOutput("CAN/TurretAlive", turret.turretAlive());
    Logger.recordOutput(
        "CAN/TurretCancoderAlive", Math.abs(turret.getTurretEncoderPosition()) > 0.0);
    Logger.recordOutput("CAN/IndexerAlive", indexer.indexerAlive());
  }
}
