package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.PhoenixUtil;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX uppies =
      new TalonFX(CLIMB_MOTOR.getDeviceNumber(), CLIMB_MOTOR.getCANBus());
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private final CANcoder climbEncoder =
      new CANcoder(CLIMB_ENCODER.getDeviceNumber(), CLIMB_ENCODER.getCANBus());
  public final int[] POWER_PORTS = {CLIMB_MOTOR.getPowerPort()};

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /** Constructor */
  public ClimbIOTalonFX() {
    /** Motion Magic Configs */
    TalonFXConfiguration uppiesConfig = new TalonFXConfiguration();
    uppiesConfig.Slot0 =
        new Slot0Configs()
            .withKP(mm_kP)
            .withKI(mm_kI)
            .withKD(mm_kD)
            .withKS(mm_kS)
            .withKV(mm_kV)
            .withKA(mm_kA);

    MotionMagicConfigs magicConfigs = uppiesConfig.MotionMagic;
    magicConfigs.MotionMagicCruiseVelocity = mm_cruiseVelocity;
    magicConfigs.MotionMagicAcceleration = mm_acceleration;
    magicConfigs.MotionMagicJerk = mm_jerk;
    uppies.getConfigurator().apply(uppiesConfig);

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    PhoenixUtil.tryUntilOk(5, () -> climbEncoder.getConfigurator().apply(cancoderConfig));
  }

  @Override
  public void setPosition(double rotations) {
    uppies.setControl(m_request.withPosition(rotations));
  }

  @Override
  public double getPosition() {
    return climbEncoder.getAbsolutePosition().getValueAsDouble();
  }
}
