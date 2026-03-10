package frc.robot.subsystems.driver_info;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldState;
import frc.robot.util.VirtualSubsystem;

public class MatchStatus extends VirtualSubsystem {

  private final CommandXboxController driver;
  private final CommandXboxController coDriver;
  Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

  public MatchStatus(CommandXboxController driver, CommandXboxController coDriver) {
    this.driver = driver;
    this.coDriver = coDriver;
  }

  public void rumble(double strength) { // makes both controllers rumble
    driver.setRumble(RumbleType.kBothRumble, strength);
    coDriver.setRumble(RumbleType.kBothRumble, strength);
  }

  public void stopRumble() { // stops both controllers rumbling
    driver.setRumble(RumbleType.kBothRumble, 0);
    coDriver.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void rbsiPeriodic() {

    if (FieldState.wonAuto == alliance) {
      if (DriverStation.getMatchTime() < 133
          && DriverStation.getMatchTime() > 130 /*deactivating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 108
          && DriverStation.getMatchTime() > 105 /*activating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 83
          && DriverStation.getMatchTime() > 80 /*deactivating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 58
          && DriverStation.getMatchTime() > 55 /*activating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 31 && DriverStation.getMatchTime() > 30) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 10
          && DriverStation.getMatchTime() > 9.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 9
          && DriverStation.getMatchTime() > 8.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 8
          && DriverStation.getMatchTime() > 7.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 7
          && DriverStation.getMatchTime() > 6.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 6
          && DriverStation.getMatchTime() > 5.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 5
          && DriverStation.getMatchTime() > 4.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 4
          && DriverStation.getMatchTime() > 3.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 3
          && DriverStation.getMatchTime() > 2.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 2
          && DriverStation.getMatchTime() > 1.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 1
          && DriverStation.getMatchTime() > 0.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else {
        stopRumble();
      }
    } else if (FieldState.wonAuto != alliance) {
      if (DriverStation.getMatchTime() < 108
          && DriverStation.getMatchTime() > 105 /*deactivating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 83
          && DriverStation.getMatchTime() > 80 /*activating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 58
          && DriverStation.getMatchTime() > 55 /*deactivating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 33
          && DriverStation.getMatchTime() > 30 /*activating hub*/) {
        rumble(0.5);
      } else if (DriverStation.getMatchTime() < 10
          && DriverStation.getMatchTime() > 9.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 9
          && DriverStation.getMatchTime() > 8.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 8
          && DriverStation.getMatchTime() > 7.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 7
          && DriverStation.getMatchTime() > 6.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 6
          && DriverStation.getMatchTime() > 5.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 5
          && DriverStation.getMatchTime() > 4.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 4
          && DriverStation.getMatchTime() > 3.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 3
          && DriverStation.getMatchTime() > 2.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 2
          && DriverStation.getMatchTime() > 1.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else if (DriverStation.getMatchTime() < 1
          && DriverStation.getMatchTime() > 0.5
          && !DriverStation.isAutonomous()) {
        rumble(0.25);
      } else {
        stopRumble();
      }
    } else {
      stopRumble(); // if no value won for who won auto the controllers will never vibrate
    }
  }
}
