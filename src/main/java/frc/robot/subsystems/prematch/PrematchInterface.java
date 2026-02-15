package frc.robot.subsystems.prematch;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;

public class PrematchInterface {

  public Turret turret;
  public Intake intake;

  public boolean checkPivotPosition() {

    this.turret = turret;
    this.intake = intake;

    if (intake.getPosition() > 0.4)
      ;
    {
      return true;


    }
    
  }

}
