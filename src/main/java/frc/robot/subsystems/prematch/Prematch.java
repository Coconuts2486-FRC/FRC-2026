package frc.robot.subsystems.prematch;

import edu.wpi.first.wpilibj.Timer;
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

  private Turret turret;
  private Intake intake;
  private boolean physicalTestIndicator = false;

  // Display timing vars
  private double indicatorEndTime;
  private static final double timerLimit = 1;

  public int stringPosition = 0;

  /**** Be careful when editing these lists. Changing one of the lists without changing the others
   * accordingly will cause this entire bit of code to break and crash.
   * Make sure you understand how these lists work. */
  public List<String> clickableInfo =
      List.of(
          "Systems Check?",
          "Replaced Battery?",
          "Bumpers Correct Color?",
          "Turret in Position?",
          "Intake in Position?");

  public List<String> physicalChecksList = List.of("Turret in Position?", "Intake in Position?");
  public List<String> humanConfirmationList =
      List.of("Systems Check", "Replaced Battery", "Bumpers Correct Color?");
  public Map<String, BooleanSupplier> physicalChecksMap =
      Map.of(
          "Turret in Position?",
          this::Turret_In_Position,
          "Intake in Position?",
          this::Intake_In_Position);

  @Override
  public void rbsiPeriodic() {

    double now = Timer.getFPGATimestamp();

    Logger.recordOutput("Prematch/Turret in Position", Turret_In_Position());

    if (now >= indicatorEndTime) {
      physicalTestIndicator = false;
    }

    if (stringPosition >= clickableInfo.size()) {
      stringPosition = 0;
    }

    if (stringPosition >= humanConfirmationList.size()) {
      Logger.recordOutput(
          "Prematch/Physical Check",
          physicalChecksMap.get(clickableInfo.get(stringPosition)).getAsBoolean());
      Logger.recordOutput("Prematch/Physical Test Switch", physicalTestIndicator);
    }

    Logger.recordOutput("Prematch/Currently Checking", clickableInfo.get(stringPosition));
    Logger.recordOutput("Prematch/Physical Test Switch", physicalTestIndicator);
  }

  // Bits of code that set up booleans for the list to read. These are private so that it doesn't
  // get mixed up with similar functions in RobotContainer.

  // this checks turret position
  private boolean Turret_In_Position() {
    return turret.readTurretSwitch();
  }

  public void enableUpdate() {
    if (physicalTestIndicator) {
      return;
    }

    physicalTestIndicator = true;
    stringPosition++;
    indicatorEndTime = Timer.getFPGATimestamp() + timerLimit;
    System.out.println("ran enableUpdate function");
  }

  // this checks intake position
  private boolean Intake_In_Position() {
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
