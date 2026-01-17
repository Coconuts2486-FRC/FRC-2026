package frc.robot.subsystems.Driver_Info;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldState;
import frc.robot.util.VirtualSubsystem;

public class match_status extends VirtualSubsystem {

  private final CommandXboxController driver;
  private final CommandXboxController coDriver;
  Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

  public match_status(CommandXboxController driver, CommandXboxController coDriver) {
    this.driver = driver;
    this.coDriver = coDriver;
  }

  public void rumble() { // makes both controllers rumble
    driver.setRumble(RumbleType.kBothRumble, 0.5);
    coDriver.setRumble(RumbleType.kBothRumble, 0.5);
  }

  public void stopRumble() { // stops both controllers rumbling
    driver.setRumble(RumbleType.kBothRumble, 0);
    coDriver.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    if (FieldState.wonAuto == alliance) {
      if (DriverStation.getMatchTime() < 133
          && DriverStation.getMatchTime() > 130 /*deactivating hub*/) {
        rumble();
      } else if (DriverStation.getMatchTime() < 108
          && DriverStation.getMatchTime() > 105 /*activating hub*/) {
        rumble();
      } else if (DriverStation.getMatchTime() < 83
          && DriverStation.getMatchTime() > 80 /*deactivating hub*/) {
        rumble();
      } else if (DriverStation.getMatchTime() < 58
          && DriverStation.getMatchTime() > 55 /*activating hub*/) {
        rumble();
      } else {
        stopRumble();
      }
    } else if (FieldState.wonAuto != alliance) {
      if (DriverStation.getMatchTime() < 108
          && DriverStation.getMatchTime() > 105 /*deactivating hub*/) {
        rumble();
      } else if (DriverStation.getMatchTime() < 83
          && DriverStation.getMatchTime() > 80 /*activating hub*/) {
        rumble();
      } else if (DriverStation.getMatchTime() < 58
          && DriverStation.getMatchTime() > 55 /*deactivating hub*/) {
        rumble();
      } else if (DriverStation.getMatchTime() < 33
          && DriverStation.getMatchTime() > 30 /*activating hub*/) {
        rumble();
      } else {
        stopRumble();
      }
    } else {
      stopRumble(); // if no value won for who won auto the controllers will never vibrate
    }
  }
}
