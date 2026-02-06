package frc.robot.subsystems.Intake;

// import static frc.robot.Constants.climbConstants.mm_acceleration;
// import static frc.robot.Constants.climbConstants.mm_cruiseVelocity;
// import static frc.robot.Constants.climbConstants.mm_jerk;
// import static frc.robot.Constants.climbConstants.mm_kA;
// import static frc.robot.Constants.climbConstants.mm_kD;
// import static frc.robot.Constants.climbConstants.mm_kI;
// import static frc.robot.Constants.climbConstants.mm_kP;
// import static frc.robot.Constants.climbConstants.mm_kS;
// import static frc.robot.Constants.climbConstants.mm_kV;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.pivotConstants.*;

public class IntakeIOTalonFX implements intakeIO {
  private final TalonFX pivot = new TalonFX(32);
  private final TalonFX rollers = new TalonFX(33);

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);

  ProfiledPIDController controller =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(5, 10));

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
  public void pivotToPos(double pos) {
    pivot.set(controller.calculate(pivotEncoder.get(), pos));
  }

  @Override
  public double getPosition() {
    return pivotEncoder.get();
  }
}
