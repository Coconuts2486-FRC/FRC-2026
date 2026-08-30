// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
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
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.imu;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;
import frc.robot.Constants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.RBSICANBusRegistry;
import java.util.Iterator;
import java.util.Queue;

/** IMU IO for CTRE Pigeon2 */
public class ImuIOPigeon2 implements ImuIO {

  // Define the Pigeon2 Hardware
  private final Pigeon2 pigeon =
      new Pigeon2(
          SwerveConstants.kPigeonId, RBSICANBusRegistry.getBus(SwerveConstants.kCANbusName));

  // Cached signals
  private final StatusSignal<Angle> yawSignal = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawRateSignal = pigeon.getAngularVelocityZWorld();

  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
  private final StatusSignal<LinearAcceleration> accelZ = pigeon.getAccelerationZ();

  private final Queue<Double> odomTimestamps;
  private final Queue<Double> odomYawsDeg;

  // Previous accel for jerk calculation (m/s/s)
  private Translation3d prevAcc = Translation3d.kZero;
  private long prevTimestampNs = 0L;

  // Reusable buffers for queue-drain (to avoid using streams)
  private double[] odomTsBuf = new double[8];
  private double[] odomYawRadBuf = new double[8];

  /** Constructor */
  public ImuIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yawSignal.setUpdateFrequency(SwerveConstants.kOdometryFrequency);
    yawRateSignal.setUpdateFrequency(50.0);

    accelX.setUpdateFrequency(50.0);
    accelY.setUpdateFrequency(50.0);
    accelZ.setUpdateFrequency(50.0);

    pigeon.optimizeBusUtilization();

    odomTimestamps = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    odomYawsDeg = PhoenixOdometryThread.getInstance().registerSignal(yawSignal);
  }

  /** Update the Inputs */
  @Override
  public void updateInputs(ImuIOInputs inputs) {
    final long start = System.nanoTime();

    // Load the nanosecond timestamp
    inputs.timestampNs = start;

    StatusCode code = BaseStatusSignal.refreshAll(yawSignal, yawRateSignal, accelX, accelY, accelZ);

    // Phoenix retains the last signal value after a CAN timeout. Do not present that stale value as
    // a new measurement, and never allow a non-finite value to reach odometry.
    final double yawRad = Units.degreesToRadians(yawSignal.getValueAsDouble());
    final double yawRateRadPerSec = Units.degreesToRadians(yawRateSignal.getValueAsDouble());
    final Translation3d accel =
        new Translation3d(
            accelX.getValueAsDouble() * Constants.G_TO_MPS2,
            accelY.getValueAsDouble() * Constants.G_TO_MPS2,
            accelZ.getValueAsDouble() * Constants.G_TO_MPS2);
    inputs.connected =
        code.isOK()
            && Double.isFinite(yawRad)
            && Double.isFinite(yawRateRadPerSec)
            && isFinite(accel);

    if (inputs.connected) {
      inputs.yawPositionRad = yawRad;
      inputs.yawRateRadPerSec = yawRateRadPerSec;
      inputs.linearAccel = accel;
    }

    // Jerk computed as (delta accel) / dt
    if (inputs.connected && prevTimestampNs != 0L) {
      final double dt = (start - prevTimestampNs) * 1e-9;
      // Only compute if `dt` is larger than 1 ms.
      if (dt > 1e-6) {
        inputs.linearJerk = inputs.linearAccel.minus(prevAcc).div(dt);
      }
    }

    // Load "previous values" for the next loop
    if (inputs.connected) {
      prevTimestampNs = start;
      prevAcc = inputs.linearAccel;
    }

    // Drain odometry queues to primitive arrays (timestamps == doubles; yaws == degrees)
    final int n = inputs.connected ? drainOdometryQueuesIntoBuffers() : drainAndDiscardOdometryQueues();
    if (n > 0) {
      // If there's anything to drain...
      final double[] tsOut = new double[n];
      final double[] yawOut = new double[n];
      System.arraycopy(odomTsBuf, 0, tsOut, 0, n);
      System.arraycopy(odomYawRadBuf, 0, yawOut, 0, n);
      inputs.odometryYawTimestamps = tsOut;
      inputs.odometryYawPositionsRad = yawOut;
    } else {
      // ...otherwise return empty arrays
      inputs.odometryYawTimestamps = EMPTY_DOUBLE_ARRAY;
      inputs.odometryYawPositionsRad = EMPTY_DOUBLE_ARRAY;
    }

    // Compute how long this took in seconds
    final long end = System.nanoTime();
    inputs.latencySeconds = (end - start) * 1e-9;
  }

  /**
   * Zero the YAW to this radian value
   *
   * @param yawRad The radian value to which to zero
   */
  @Override
  public void zeroYawRad(double yawRad) {
    pigeon.setYaw(Units.radiansToDegrees(yawRad));
  }

  /**
   * Drain the Odometry Queues into a Buffer
   *
   * <p>Private function that does the heavy lifting of draining the queues
   */
  private int drainOdometryQueuesIntoBuffers() {
    final int nTs = odomTimestamps.size();
    final int nYaw = odomYawsDeg.size();
    final int n = Math.min(nTs, nYaw);
    if (n <= 0) {
      odomTimestamps.clear();
      odomYawsDeg.clear();
      return 0;
    }

    ensureOdomCapacity(n);

    // Iterate without streams (still boxed because Queue<Double>, but avoids stream overhead)
    final Iterator<Double> itT = odomTimestamps.iterator();
    final Iterator<Double> itY = odomYawsDeg.iterator();

    int i = 0;
    while (i < n && itT.hasNext() && itY.hasNext()) {
      final Double timestamp = itT.next();
      final Double yawDeg = itY.next();
      final double yawRad = yawDeg == null ? Double.NaN : Units.degreesToRadians(yawDeg);
      if (timestamp != null && Double.isFinite(timestamp) && Double.isFinite(yawRad)) {
        odomTsBuf[i] = timestamp;
        odomYawRadBuf[i] = yawRad;
        i++;
      }
    }

    odomTimestamps.clear();
    odomYawsDeg.clear();
    return i;
  }

  /** Clears queued samples after a failed refresh so stale data is never replayed as a new sample. */
  private int drainAndDiscardOdometryQueues() {
    odomTimestamps.clear();
    odomYawsDeg.clear();
    return 0;
  }

  private static boolean isFinite(Translation3d value) {
    return Double.isFinite(value.getX())
        && Double.isFinite(value.getY())
        && Double.isFinite(value.getZ());
  }

  /**
   * Check that buffer is big enough for this queue
   *
   * <p>Private function that ensures odometry buffer capacity
   */
  private void ensureOdomCapacity(int n) {
    if (odomTsBuf.length >= n) return;
    int newCap = odomTsBuf.length;
    while (newCap < n) newCap *= 2;
    odomTsBuf = new double[newCap];
    odomYawRadBuf = new double[newCap];
  }

  /** Dummy function to make things happy -- doesn't actually do anything */
  @Override
  public int[] powerPorts() {
    return new int[] {};
  }
}
