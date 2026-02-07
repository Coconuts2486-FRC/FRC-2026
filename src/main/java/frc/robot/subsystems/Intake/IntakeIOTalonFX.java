package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.pivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.intakeConstants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class IntakeIOTalonFX implements intakeIO {

  // Declare Hardware
  private final TalonFX pivot =
      new TalonFX(INTAKE_PIVOT.getDeviceNumber(), INTAKE_PIVOT.getCANBus());
  private final TalonFX rollers =
      new TalonFX(INTAKE_ROLLER.getDeviceNumber(), INTAKE_ROLLER.getCANBus());

  public final int[] powerPorts = {INTAKE_PIVOT.getPowerPort(), INTAKE_ROLLER.getPowerPort()};

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2); // returns position 0-1

  final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  /** Constuctor */
  public IntakeIOTalonFX() {
    config.Slot0 = new Slot0Configs().withKP(1.0);

    PhoenixUtil.tryUntilOk(5, () -> pivot.getConfigurator().apply(config, 0.25));
  }

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

  /** Set the mechanism angular velocity in physical units */
  @Override
  public void setPivotVelocity(AngularVelocity velocity) {
    //
    var motorVelocity = velocity.in(RotationsPerSecond) * intakeConstants.kPivotGearRatio;
    Logger.recordOutput("Intake/Pivot/requestedVelocity", velocity);
    Logger.recordOutput("Intake/Pivot/motorVelocity", motorVelocity);
    pivot.setControl(velocityRequest.withVelocity(motorVelocity));
  }

  @Override
  public void setRollerVelocity(double velocity) {
    rollers.set(velocity);
  }

  @Override
  public Angle getPosition() {
    // The encoder returns position in units of rotations
    return Rotations.of(pivotEncoder.get());
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
