package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.intakeConstants;
import frc.robot.util.RBSISubsystem;

public class intake extends RBSISubsystem {
  private intakeIO io;
  private final intakeIOInputsAutoLogged inputs = new intakeIOInputsAutoLogged();

  // public variables to be pulled by can class
  public static boolean pivotAlive;
  public static boolean rollersAlive;

  // Constructor
  public intake(intakeIO io) {
    this.io = io;
  }

  // max values are for rotations a second
  ProfiledPIDController controller =
      new ProfiledPIDController(
          intakeConstants.kp.get(),
          intakeConstants.ki,
          intakeConstants.kd,
          new TrapezoidProfile.Constraints(
              intakeConstants.maxVelocity.get(), intakeConstants.maxAcel.get()));

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

  public void setPivotVelocity(AngularVelocity velocity) {
    io.setPivotVelocity(velocity);
  }

  // gives the intake a little push but then lets it fall down and be free while intaking
  public void pivotDown() {
    if (io.getPosition().in(Rotations) < intakeConstants.dropPostion.in(Rotations)) {
      io.setPivotVelocity(RotationsPerSecond.of(0.75));
      io.setRollerVelocity(0.65);
    } else {
      io.stopPivot();
      io.setRollerVelocity(0.65);
    }
  }

  // brings pivot up with pid while running intake motors still, stopping themif at position
  public void pivotUp() {
    if (io.getPosition() != intakeConstants.storedAngle) {
      io.setPivotVelocity(
          RotationsPerSecond.of(
              controller.calculate(
                  io.getPosition().in(Rotations), intakeConstants.storedAngle.in(Rotations))));
      io.setRollerVelocity(0.65);
    } else if (io.getPosition() == intakeConstants.storedAngle) {
      io.setPivotVelocity(
          RotationsPerSecond.of(
              controller.calculate(
                  io.getPosition().in(Rotations), intakeConstants.storedAngle.in(Rotations))));
      io.stopRoller();
    }
  }

  public void pivotGoToPosition(double pos) {
    io.setPivotVelocity(
        RotationsPerSecond.of(controller.calculate(io.getPosition().in(Rotations), pos)));
  }

  public Angle getPosition() {
    return io.getPosition();
  }

  public void stopRoller() {
    io.stopRoller();
  }

  public void stopPivot() {
    io.stopPivot();
  }
}
