// Copyright 2026 FRC 2486
// https://github.com/Coconuts2486-FRC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DriveTest {
  private static final double EPSILON = 1e-9;

  @Test
  void yawRateUsesShortestAngleAcrossWraparound() {
    double previousYaw = Math.toRadians(179.0);
    double yaw = Math.toRadians(-179.0);

    assertEquals(Math.toRadians(20.0), Drive.calculateYawRate(previousYaw, yaw, 0.1), EPSILON);
  }
}
