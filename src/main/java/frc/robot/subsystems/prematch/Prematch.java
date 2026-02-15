package frc.robot.subsystems.prematch;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.turret.*;
import frc.robot.util.RBSISubsystem;
import java.util.List;
import org.littletonrobotics.junction.*;

public class Prematch extends RBSISubsystem {

  private PrematchInterface io;

  public Prematch(Turret turret, PrematchInterface io) {
    this.turret = turret;
    this.io = io;
  }

  public SmartDashboard dashboard;

  public Turret turret;

  public List<String> clickableInfo = List.of("Systems_Check", "Replaced_Battery");
  public List<String> robotInfoText = List.of("Turrets_In_Position", "Pivot_In_Position");
  public int stringPosition = 0;

  @Override
  public void rbsiPeriodic() {

    Logger.recordOutput("Turret at Zero Point", turret.readTurretSwitch());
  }

  public void checklist(Boolean conditionIsTrue) {

    if (stringPosition < clickableInfo.size()) {
      Logger.recordMetadata("Currently Checking", clickableInfo.get(stringPosition));

      if (conditionIsTrue) {
        updateChecklist();
      }

    } else if (stringPosition < robotInfoText.size() + clickableInfo.size()) {
      Logger.recordMetadata(
          "Currently Checking", robotInfoText.get(stringPosition - clickableInfo.size()));

      if (conditionIsTrue) {
        updateChecklist();
      }
    }
  }

  public void updateChecklist() {
    stringPosition++;

    if (stringPosition >= robotInfoText.size() + clickableInfo.size()) {
      stringPosition = 0;
    }
  }
}
