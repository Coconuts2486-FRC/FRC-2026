package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX uppies = new TalonFX(41);

  public ClimbIOTalonFX() {
    /** Motion Magic Configs */
    TalonFXConfiguration uppiesConfig = new TalonFXConfiguration();
    uppiesConfig.Slot0 =
        new Slot0Configs()
            .withKP(0.25)
            .withKI(0.0)
            .withKD(0.25)
            .withKS(0.25)
            .withKV(0.25)
            .withKA(0.25);
  }

  private final DutyCycleEncoder uppiesEncoder = new DutyCycleEncoder(3);
}
