package frc.robot.subsystems.turret;

import static frc.robot.Constants.RobotDevices.TURRET_ENCODER;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.TurretConstants;

public class TurretIOTalonFX implements TurretIO {

  private DigitalInput turretSwitch = new DigitalInput(0);
  private CANcoder turretEncoder =
      new CANcoder(TURRET_ENCODER.getDeviceNumber(), TURRET_ENCODER.getCANBus());

  // Constructor
  public TurretIOTalonFX() {

    turretEncoder = new CANcoder(TurretConstants.encoderID);
  }

  @Override
  public boolean readTurretSwitch() {
    return turretSwitch.get();
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
