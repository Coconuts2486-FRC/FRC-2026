package frc.robot.subsystems.climb;

import static frc.robot.Constants.climbConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX uppies = new TalonFX(41);
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

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
  }

  @Override
  public void setPosition(double rotations) {
    uppies.setControl(m_request.withPosition(rotations));
  }

  @Override
  public void getEncoderPos() {
    System.out.println();
  }
}
