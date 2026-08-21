// Copyright 2026 FRC 2486
// https://github.com/Coconuts2486-FRC

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class RBSIControllerTest {
  @Test
  void detectsXboxFromDriverStationFlag() {
    assertEquals("Xbox", RBSIController.detectControllerType(true, "Wireless Controller"));
  }

  @Test
  void detectsPlayStationControllerFamiliesByName() {
    assertEquals(
        "PS5", RBSIController.detectControllerType(false, "DualSense Wireless Controller"));
    assertEquals("PS5", RBSIController.detectControllerType(false, "PS5 Controller"));
    assertEquals("PS4", RBSIController.detectControllerType(false, "DualShock 4"));
    assertEquals("PS4", RBSIController.detectControllerType(false, "PlayStation Controller"));
  }

  @Test
  void unknownAndMissingNamesUseXboxCompatibleFallback() {
    assertEquals("Xbox", RBSIController.detectControllerType(false, "Generic USB Gamepad"));
    assertEquals("Xbox", RBSIController.detectControllerType(false, null));
  }
}
