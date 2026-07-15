// Copyright 2026 FRC 2486
// https://github.com/Coconuts2486-FRC
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.OperatorConstants;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void linearVelocityAppliesDeadbandAndShapesMagnitudeWithoutChangingDirection() {
    assertEquals(Translation2d.kZero, DriveCommands.getLinearVelocity(0.01, 0.01));

    Translation2d velocity = DriveCommands.getLinearVelocity(0.3, 0.4);
    double scaledMagnitude = MathUtil.applyDeadband(0.5, OperatorConstants.kDeadband);
    double shapedMagnitude =
        Math.pow(scaledMagnitude, OperatorConstants.kLinearJoystickResponseExponent);

    assertEquals(shapedMagnitude, velocity.getNorm(), EPSILON);
    assertEquals(Math.atan2(0.4, 0.3), velocity.getAngle().getRadians(), EPSILON);
  }

  @Test
  void omegaAppliesDeadbandAndPreservesSignWhenShaped() {
    double scaled = MathUtil.applyDeadband(0.5, OperatorConstants.kDeadband);
    double shaped = Math.pow(scaled, OperatorConstants.kAngularJoystickResponseExponent);

    assertEquals(0.0, DriveCommands.getOmega(0.01), EPSILON);
    assertEquals(shaped, DriveCommands.getOmega(0.5), EPSILON);
    assertEquals(-shaped, DriveCommands.getOmega(-0.5), EPSILON);
  }
}
