package frc.robot.subsystems.prematch;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.turret.*;
import frc.robot.util.RBSISubsystem;
import java.util.List;
import org.littletonrobotics.junction.*;

public class Prematch extends RBSISubsystem {

  

  public Prematch(Turret turret, Intake intake) {
    this.turret = turret;
    this.intake = intake;
  }

  public SmartDashboard dashboard;

  public Turret turret;
  public Intake intake;

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

  // Bits of code that set up booleans for the list to read. These are private so that it doesn't
  // get mixed up with similar functions in RobotContainer.
  private Boolean goodTurretPosition() {
    return turret.readTurretSwitch();
  }

  private Boolean goodPivotPosition() {
    if (Constants.intakeConstants.storedAngle < (intake.getPosition() + 0.02)) {
      if (Constants.intakeConstants.storedAngle > (intake.getPosition() - 0.02)) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}
