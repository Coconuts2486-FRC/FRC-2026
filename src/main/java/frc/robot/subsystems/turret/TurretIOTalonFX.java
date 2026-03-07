package frc.robot.subsystems.turret;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.TurretConstants;

public class TurretIOTalonFX implements TurretIO {

  // Declare Hardware
  private final TalonFX turret =
      new TalonFX(TURRET_POINTER.getDeviceNumber(), TURRET_POINTER.getCANBus());
  private DigitalInput turretSwitch = new DigitalInput(0);
  private CANcoder turretEncoder =
      new CANcoder(TURRET_ENCODER.getDeviceNumber(), TURRET_ENCODER.getCANBus());
  public final int[] POWER_PORTS = {TURRET_POINTER.getPowerPort()};

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /** Constructor */
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
