package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class TurretIOTalonFX implements TurretIO {

  private DigitalInput limitSwitch = new DigitalInput(0);
  private CANcoder turretEncoder = new CANcoder(43);

  // Constructor
  public TurretIOTalonFX() {}

  @Override
  public boolean readTurretSwitch() {
    return limitSwitch.get();
  }

  @Override
  public double getTurretEncoderPosition() {
    return turretEncoder.getPosition().getValueAsDouble();
  }

  @Override
  public void zeroEncoder() {
    turretEncoder.setPosition(0.0);
  }
}
