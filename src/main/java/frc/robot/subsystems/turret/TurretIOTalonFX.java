package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DigitalInput;

public class TurretIOTalonFX implements TurretIO {

  private DigitalInput limitSwitch = new DigitalInput(0);

  // Constructor
  public TurretIOTalonFX() {}

  @Override
  public boolean readTurretSwitch() {
    return limitSwitch.get();
  }
}
