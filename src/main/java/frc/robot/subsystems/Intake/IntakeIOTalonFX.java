package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivot = new TalonFX(32);
  private final TalonFX rollers = new TalonFX(33);

  private final PIDController pivotPID = new PIDController(1.5, 0, 0);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);

  public IntakeIOTalonFX() {}

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

  /** Return the current encoder value for the intake pivot */
  @Override
  public double getPosition() {
    return pivotEncoder.get();
  }
}
