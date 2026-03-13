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

  /** Constructor */
  public MatchStatus(CommandXboxController driver, CommandXboxController coDriver) {
    this.driver = driver;
    this.coDriver = coDriver;
  }

  /**
   * Make both controllers rumble
   *
   * @param strength Rumble strength
   */
  public void rumble(double strength) {
    driver.setRumble(RumbleType.kBothRumble, strength);
    coDriver.setRumble(RumbleType.kBothRumble, strength);
  }

  /** Make both controllers stop rumbling */
  public void stopRumble() {
    driver.setRumble(RumbleType.kBothRumble, 0);
    coDriver.setRumble(RumbleType.kBothRumble, 0);
  }

  /** Periodic function */
  @Override
  public void rbsiPeriodic() {

    // Do these deeper calls once, and use the values for the rest of this loop
    double matchTime = DriverStation.getMatchTime();
    if (DriverStation.isAutonomous()) {
      // If Auto, don't do anything for this
      return;
    }

    // Alliance-Specific HUB Activation / Deactivation
    // Rumble & return
    if (FieldState.wonAuto == alliance) {
      if (matchTime < 133 && matchTime > 130) {
        // Deactivating HUB
        rumble(0.5);
        return;
      }
      if (matchTime < 108 && matchTime > 105) {
        // Activating HUB
        rumble(0.5);
        return;
      }
      if (matchTime < 83 && matchTime > 80) {
        // Deactivating HUB
        rumble(0.5);
        return;
      }
      if (matchTime < 58 && matchTime > 55) {
        // Activating HUB
        rumble(0.5);
        return;
      }

    } else if (FieldState.wonAuto != alliance) {
      if (matchTime < 108 && matchTime > 105) {
        // Deactivating HUB
        rumble(0.5);
        return;
      }
      if (matchTime < 83 && matchTime > 80) {
        // Activating HUB
        rumble(0.5);
        return;
      }
      if (matchTime < 58 && matchTime > 55) {
        // Deactivating HUB
        rumble(0.5);
        return;
      }
      if (matchTime < 33 && matchTime > 31) {
        // Activating HUB
        rumble(0.5);
        return;
      }
    }

    // Endgame Rumble -- same regardless
    if (matchTime < 31 && matchTime > 30) {
      rumble(0.25);
      return;
    }
    // Rumble every second during the last 10 seconds of the match
    if (matchTime < 10 && isUpperHalfSecond(matchTime)) {
      rumble(0.25);
      return;
    }

    // If we get all the way here, stop rumble!
    stopRumble(); // if no value won for who won auto the controllers will never vibrate
  }

  /**
   * Check that the time is in the "upper half" of the second
   *
   * @param time The matchTime
   */
  private boolean isUpperHalfSecond(double time) {

    // The midpoint is halfway between the floor and ceiling
    double midpoint = (double) (Math.floor(time) + Math.ceil(time)) / 2.0;
    return time > midpoint;
  }
}
