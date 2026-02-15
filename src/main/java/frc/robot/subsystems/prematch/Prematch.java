package frc.robot.subsystems.prematch;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.turret.*;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.*;

public class Prematch extends RBSISubsystem {

  public Prematch() {}

  public SmartDashboard dashboard;

  public Turret turret;

  public String info;
  public int position;

  @Override
  public void rbsiPeriodic() {

    Logger.recordOutput("Turret at Zero Point", turret.readTurretSwitch());
  }


  public void checklist(){

  }
}
