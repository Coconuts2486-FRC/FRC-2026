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
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Prematch extends RBSISubsystem {

  public Prematch(Turret turret, Intake intake) {
    this.turret = turret;
    this.intake = intake;
  }

  public SmartDashboard dashboard;

  public Turret turret;
  public Intake intake;
  public Boolean enableUpdate = false;

  public int var1 = 1;
  public int var2 = 2;

  public List<String> clickableInfo = List.of("Systems_Check", "Replaced_Battery");
  public List<String> physicalChecksList = List.of("Turret in Position", "Intake_In_Position");
  public Map<String, BooleanSupplier> physicalChecksMap =
      Map.of(
          "Turret_In_Position",
          this::Turret_In_Position,
          "Intake_In_Position",
          this::Intake_In_Position);
  public int stringPosition = 1;

  @Override
  public void rbsiPeriodic() {

    if (var1 < var2) {

      Logger.recordOutput(
          "Prematch/Currently Checking", String.valueOf(physicalChecksList.get(stringPosition)));

    } else if (stringPosition < (physicalChecksList.size() + clickableInfo.size())) {
      Logger.recordOutput(
          "Prematch/Currently Checking",
          String.valueOf(physicalChecksList.get(stringPosition - clickableInfo.size())));
    }

    Logger.recordOutput("Prematch/Turret in Position", !Turret_In_Position());
    Logger.recordOutput("Prematch/Currently Checking 2", physicalChecksList.get(stringPosition));
  }

  public void enableUpdate() {

    enableUpdate = true;
    updateChecklist();
    enableUpdate = false;
  }

  public void updateChecklist() {

    if (enableUpdate) {
      stringPosition++;
    }
    if (stringPosition >= physicalChecksMap.size() + clickableInfo.size()) {
      stringPosition = 0;
    }
  }

  // Bits of code that set up booleans for the list to read. These are private so that it doesn't
  // get mixed up with similar functions in RobotContainer.

  // this checks turret position
  private Boolean Turret_In_Position() {
    return turret.readTurretSwitch();
  }

  // this checks intake position
  private Boolean Intake_In_Position() {
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
