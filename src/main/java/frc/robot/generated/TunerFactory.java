package frc.robot.generated;

import frc.robot.Constants;

public final class TunerFactory {
  public static final TunerView INSTANCE = create();

  private static TunerView create() {
    return switch (Constants.getRobot()) {
      case PINCHY -> new PinchyTunerView();
      case GEORGE -> new GeorgeTunerView();
      case COMPBOT -> new CompbotTunerView();
      default -> new CompbotTunerView();
    };
  }
}
