package frc.robot.subsystems.intake;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.intakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  // Declare Hardware
  private final TalonFX pivot =
      new TalonFX(INTAKE_PIVOT.getDeviceNumber(), INTAKE_PIVOT.getCANBus());
  private final TalonFX rollers =
      new TalonFX(INTAKE_ROLLER.getDeviceNumber(), INTAKE_ROLLER.getCANBus());

  public final int[] powerPorts = {INTAKE_PIVOT.getPowerPort(), INTAKE_ROLLER.getPowerPort()};

  private final CANcoder pivotEncoder;

  public IntakeIOTalonFX() {

    pivotEncoder = new CANcoder(intakeConstants.encoderID);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
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

  /** Set the mechanism angular velocity in physical units */
  @Override
  public void setPivotVelocity(double velocity) {
    pivot.set(velocity);
  }

  @Override
  public void setRollerVelocity(double velocity) {
    rollers.set(velocity);
  }

  @Override
  public boolean isIntakeRollersRunning() {
    if (rollers.get() > 0.02) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public boolean isIntakeExtended() {
    if (pivotEncoder.getAbsolutePosition().getValueAsDouble()
        > (intakeConstants.dropPostion - 0.05)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public double getPosition() {
    // The encoder returns position in units of rotations
    return pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void stopPivot() {
    pivot.stopMotor();
    rollers.stopMotor();
  }

  @Override
  public void stopRoller() {
    rollers.stopMotor();
  }
}
