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

import static frc.robot.Constants.pivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOTalonFX implements intakeIO {
  private final TalonFX pivot = new TalonFX(32);
  private final TalonFX rollers = new TalonFX(33);

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2); // returns position 0-1

  public IntakeIOTalonFX() {}

  @Override
  public void updateInputs(intakeIOInputs inputs) {
    // checks the status of pivot
    var pivotStatus =
        BaseStatusSignal.refreshAll(
            pivot.getStatorCurrent(),
            pivot.getPosition(),
            pivot.getVelocity(),
            pivot.getMotorVoltage());

    // checks the status of roller
    var rollerStatus =
        BaseStatusSignal.refreshAll(
            rollers.getStatorCurrent(),
            rollers.getPosition(),
            rollers.getVelocity(),
            rollers.getMotorVoltage());

    inputs.pivotConnected = pivotStatus.isOK();
    inputs.rollerConnected = rollerStatus.isOK();
  }

  @Override
  public void setPivotVelocity(double velocity) {
    pivot.set(velocity);
  }

  @Override
  public void setRollerVelocity(double velocity) {
    rollers.set(velocity);
  }

  @Override
  public double getPosition() {
    return pivotEncoder.get();
  }

  @Override
  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void stopRoller() {
    rollers.stopMotor();
  }
}
