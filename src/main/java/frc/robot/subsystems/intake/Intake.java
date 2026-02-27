package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.intakeConstants;
import frc.robot.util.RBSISubsystem;

public class Intake extends RBSISubsystem {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // public variables to be pulled by can class
  public static boolean pivotAlive;
  public static boolean rollersAlive;

  public static boolean intaking;

  // Constructor
  public Intake(IntakeIO io) {
    this.io = io;

    // setDefaultCommand(Commands.run(() -> pivotUp(), this));
  }

  // max values are for rotations a second
  PIDController controller =
      new PIDController(intakeConstants.kp.get(), intakeConstants.ki, intakeConstants.kd);

  @Override
  public void simulationPeriodic() {}

  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);

    // updates public variables checking to see if they die
    rollersAlive = inputs.rollerConnected;
    pivotAlive = inputs.pivotConnected;
  }

  // prints the encoder position temporary testing function
  public void print() {
    System.out.println(io.getPosition());
  }

  public void setRollerVelocity(double velocity) {
    io.setRollerVelocity(velocity);
  }

  public void setPivotVelocity(double velocity) {
    io.setPivotVelocity(velocity);
  }

  // gives the intake a little push but then lets it fall down and be free while intaking
  public void pivotDown() {
    if (io.getPosition() < intakeConstants.dropPostion) {
      io.setPivotVelocity(0.75);
      // io.setRollerVelocity(0.65);
    } else {
      io.stopPivot();
      // io.setRollerVelocity(0.65);
    }
  }

  public void runRollers() {
    io.setRollerVelocity(0.65);
  }

  public void stopRollers() {
    io.stopRoller();
  }

  // brings pivot up with pid while running intake motors still, stopping themif at position
  public void pivotUp() {
    if (io.getPosition() < intakeConstants.storedAngle + 0.05
        && io.getPosition() > intakeConstants.storedAngle - 0.05) {
      io.setPivotVelocity(controller.calculate(io.getPosition(), intakeConstants.storedAngle));

      io.stopRoller();
    } else {

      io.setPivotVelocity(controller.calculate(io.getPosition(), intakeConstants.storedAngle));
      // io.setRollerVelocity(0.65);
    }
  }

  public void pivotGoToPosition(double pos) {
    io.setPivotVelocity(controller.calculate(io.getPosition(), pos));
  }

  public double getPosition() {
    return io.getPosition();
  }

  public void stopRoller() {
    io.stopRoller();
  }

  public void stopPivot() {
    io.stopPivot();
  }

  public boolean isIntakeRollersRunning() {
    return io.isIntakeRollersRunning();
  }

  public boolean isIntakeExtended() {
    return io.isIntakeExtended();
  }
}
