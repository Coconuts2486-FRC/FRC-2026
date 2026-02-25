package frc.robot.subsystems.feeder;

import frc.robot.util.RBSISubsystem;

public class Feeder extends RBSISubsystem {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  public void feederSetVelocity(double velocity) {
    io.feederSetVelocity(velocity);
  }

  public void feederStop() {
    io.feederStop();
  }

  public double getFeederspeed() {
    return io.getFeederspeed();
  }

  public void stop() {
    io.stop();
  }

  public boolean isFeederRunning() {
    return io.isFeederRunning();
  }

  @Override
  public void rbsiPeriodic() {}

  @Override
  public void simulationPeriodic() {}
}
