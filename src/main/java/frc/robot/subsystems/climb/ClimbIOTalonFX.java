package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX pivot = new TalonFX(67);
  private final TalonFX uppies = new TalonFX(41);

  // private final PIDController pivotPID = new PIDController(1.5, 0, 0);
  private final PIDController uppiesPID = new PIDController(1.5, 0, 0);
  // private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);
  private final DutyCycleEncoder uppiesEncoder = new DutyCycleEncoder(3);

  public ClimbIOTalonFX() {}

  // @Override
  // public void pivotToPos(double pos) {
  //   if (pos > pivotEncoder.get() + 0.1) {
  //     pivot.set(-0.2);

  //   } else if (pos < pivotEncoder.get() - 0.1) {
  //     pivot.set(0.2);
  //   } else {
  //     pivot.set(pivotPID.calculate(pivotEncoder.get(), pos));
  //   }
  // }

  @Override
  public void extendToPos(double pos) {
    if (pos > (uppiesEncoder.get() + 0.1)) {
      uppies.set(-0.2);
    } else if (pos < (uppiesEncoder.get() - 0.1)) {
      uppies.set(0.2);

    } else {
      uppies.set(uppiesPID.calculate(uppiesEncoder.get(), pos));
    }
  }
}
