package frc.robot.subsystems.intake;

import static frc.robot.Constants.climbConstants.mm_acceleration;
import static frc.robot.Constants.climbConstants.mm_cruiseVelocity;
import static frc.robot.Constants.climbConstants.mm_jerk;
import static frc.robot.Constants.climbConstants.mm_kA;
import static frc.robot.Constants.climbConstants.mm_kD;
import static frc.robot.Constants.climbConstants.mm_kI;
import static frc.robot.Constants.climbConstants.mm_kP;
import static frc.robot.Constants.climbConstants.mm_kS;
import static frc.robot.Constants.climbConstants.mm_kV;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.pivotConstants.*;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivot = new TalonFX(32);
  private final TalonFX rollers = new TalonFX(33);

  private final PIDController pivotPID = new PIDController(1.5, 0, 0);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  @Override
  public void setPivotVelocity(double velocity) {
    pivot.set(velocity);
  }

  @Override
  public void setRollerVelocity(double velocity) {
    rollers.set(velocity);
  }

  @Override
  public void configPID(double kP, double kI, double kD) {
    pivotPID.setP(kP);
    pivotPID.setI(kI);
    pivotPID.setD(kD);
  }

  @Override
  public void pivotToPos(double pos) {
    if (pos > pivotEncoder.get() + 0.1) {
      pivot.set(-0.2);

    } else if (pos < pivotEncoder.get() - 0.1) {
      pivot.set(0.2);
    } else {
      pivot.set(pivotPID.calculate(pivotEncoder.get(), pos));
    }
  }

  public IntakeIOTalonFX() {
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
    pivot.getConfigurator().apply(uppiesConfig);
  }

  @Override
  public void setMMagicPosition(double rotations) {
    pivot.setControl(m_request.withPosition(rotations));
  }

  /** Return the current encoder value for the intake pivot */
  @Override
  public void getPosition() {
    System.out.println(pivotEncoder.get());
  }
}
