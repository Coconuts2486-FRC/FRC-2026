package frc.robot.generated;

import frc.robot.Constants;

public final class TunerFactory {

  public static final TunerConstants INSTANCE = create();

  private static TunerConstants create() {
    return switch (Constants.getRobot()) {
      case PINCHY -> new PinchyTunerConstants();
      case GEORGE -> new GeorgeTunerConstants();
      case COMPBOT -> new CompbotTunerConstants();
      default -> new CompbotTunerConstants();
    };
  }
}
