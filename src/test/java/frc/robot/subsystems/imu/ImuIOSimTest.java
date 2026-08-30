// Copyright 2026 FRC 2486
// https://github.com/Coconuts2486-FRC

package frc.robot.subsystems.imu;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class ImuIOSimTest {
  private static final double EPSILON = 1e-9;

  @Test
  void odometrySamplesDrainOncePerUpdate() {
    ImuIOSim io = new ImuIOSim();
    ImuIO.ImuIOInputs inputs = new ImuIO.ImuIOInputs();

    io.simulationSetYawRad(1.0);
    io.updateInputs(inputs);
    assertEquals(1, inputs.odometryYawTimestamps.length);
    assertEquals(1, inputs.odometryYawPositionsRad.length);
    assertEquals(1.0, inputs.odometryYawPositionsRad[0], EPSILON);

    io.simulationSetYawRad(2.0);
    io.updateInputs(inputs);
    assertEquals(1, inputs.odometryYawTimestamps.length);
    assertEquals(2.0, inputs.odometryYawPositionsRad[0], EPSILON);
  }

  @Test
  void zeroYawClearsOldOdometrySamplesBeforeNextUpdate() {
    ImuIOSim io = new ImuIOSim();
    ImuIO.ImuIOInputs inputs = new ImuIO.ImuIOInputs();

    io.simulationSetYawRad(1.0);
    io.updateInputs(inputs);
    io.zeroYawRad(0.25);
    io.updateInputs(inputs);

    assertTrue(inputs.connected);
    assertEquals(0.25, inputs.yawPositionRad, EPSILON);
    assertEquals(1, inputs.odometryYawPositionsRad.length);
    assertEquals(0.25, inputs.odometryYawPositionsRad[0], EPSILON);
  }

  @Test
  void imuRetainsLastYawWhenIoReportsInvalidYaw() {
    class MutableImuIO implements ImuIO {
      double yawRad = 0.75;
      boolean connected = true;

      @Override
      public void updateInputs(ImuIOInputs inputs) {
        inputs.timestampNs++;
        inputs.connected = connected;
        inputs.yawPositionRad = yawRad;
      }
    }

    MutableImuIO io = new MutableImuIO();
    Imu imu = new Imu(io);
    imu.rbsiPeriodic();
    assertEquals(Rotation2d.fromRadians(0.75), imu.getYaw());

    io.yawRad = Double.NaN;
    imu.rbsiPeriodic();
    assertEquals(Rotation2d.fromRadians(0.75), imu.getYaw());
  }
}
