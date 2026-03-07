package frc.robot.subsystems.feeder;

import frc.robot.util.RBSISubsystem;

public class Feeder extends RBSISubsystem {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  public void setFeederVelocity(double velocity) {
    io.setFeederVelocity(velocity);
  }

  public void runFeeder() {
    io.setFeederVelocity(0.15);
  }

  public void stopFeeder() {
    io.stopFeeder();
  }

  public double getFeederspeed() {
    return io.getFeederspeed();
  }

  public boolean isFeederRunning() {
    return io.isFeederRunning();
  }

  public boolean isFeederAlive(){
    return inputs.feederAlive;
  }

  @Override
  public void rbsiPeriodic() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
