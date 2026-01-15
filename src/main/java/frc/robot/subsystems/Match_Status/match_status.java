package frc.robot.subsystems.Match_Status;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldState;
import frc.robot.util.VirtualSubsystem;

public class match_status extends VirtualSubsystem {

  private final CommandXboxController driver;
  private final CommandXboxController coDriver;

  public match_status(CommandXboxController driver, CommandXboxController coDriver) {
    this.driver = driver;
    this.coDriver = coDriver;
  }

  @Override
  public void periodic() {
    if (DriverStation.isTeleopEnabled()) {
      if (FieldState.wonAuto == DriverStation.Alliance.Red) {
        driver.setRumble(RumbleType.kRightRumble, 0.5);
        // driver.setRumble(RumbleType.kRightRumble, 0);
        // Commands.waitSeconds(0.25);
        // driver.setRumble(RumbleType.kRightRumble, 1);
        // Commands.waitSeconds(0.75);
        // driver.setRumble(RumbleType.kRightRumble, 0);
        // Commands.waitSeconds(0.25);
        // driver.setRumble(RumbleType.kRightRumble, 1);
        // Commands.waitSeconds(0.75);
        // driver.setRumble(RumbleType.kRightRumble, 0);
        // Commands.waitSeconds(0.25);
      } else {
        driver.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      driver.setRumble(RumbleType.kRightRumble, 0);
    }
  }
}
