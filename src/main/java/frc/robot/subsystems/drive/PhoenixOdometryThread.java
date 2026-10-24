// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import org.wpilib.units.measure.Angle;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.RobotController;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>Samples Phoenix 6 signals across multiple CAN networks. Signals are refreshed in groups by bus
 * because Phoenix bulk refresh calls cannot mix networks.
 */
public class PhoenixOdometryThread extends Thread {
  private static final double MAX_VALID_CAN_LATENCY_SECONDS = 0.1;
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final Map<String, List<BaseStatusSignal>> phoenixSignalsByBus = new LinkedHashMap<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<LatestSampleQueue<Double>> phoenixQueues = new ArrayList<>();
  private final List<LatestSampleQueue<Double>> genericQueues = new ArrayList<>();
  private final List<LatestSampleQueue<Double>> timestampQueues = new ArrayList<>();

  private static PhoenixOdometryThread instance = null;

  private long droppedSamples = 0;
  private long loopCount = 0;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal, String busName) {
    LatestSampleQueue<Double> queue = new LatestSampleQueue<>(128);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixSignalsByBus.computeIfAbsent(busName, name -> new ArrayList<>()).add(signal);
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    LatestSampleQueue<Double> queue = new LatestSampleQueue<>(128);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    LatestSampleQueue<Double> queue = new LatestSampleQueue<>(128);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Refresh each CAN bus separately; CTRE bulk calls require a single network.
      signalsLock.lock();
      try {
        Thread.sleep((long) (1000.0 / SwerveConstants.kOdometryFrequency));
        for (List<BaseStatusSignal> busSignals : phoenixSignalsByBus.values()) {
          BaseStatusSignal.refreshAll(busSignals);
        }
      } catch (InterruptedException e) {
        DriverStationErrors.reportWarning("Phoenix odometry thread interrupted", e.getStackTrace());
        Thread.currentThread().interrupt();
        return;
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      Drive.odometryLock.lock();
      try {
        // Sample timestamp is current FPGA time minus average CAN latency
        //     Default timestamps from Phoenix are NOT compatible with
        //     FPGA timestamps, this solution is imperfect but close
        double timestamp = RobotController.getTime() / 1e6;
        double totalLatency = 0.0;
        int validLatencyCount = 0;
        for (BaseStatusSignal signal : phoenixSignals) {
          double latency = signal.getTimestamp().getLatency();
          if (signal.getStatus().isOK() && isValidCanLatency(latency)) {
            totalLatency += latency;
            validLatencyCount++;
          }
        }
        if (validLatencyCount > 0) {
          timestamp -= totalLatency / validLatencyCount;
        }
        Logger.recordOutput("Drive/OdomThread/ValidLatencySignals", validLatencyCount);

        // Add new samples to queues
        for (int i = 0; i < phoenixSignals.length; i++) {
          if (phoenixQueues.get(i).offerLatest(phoenixSignals[i].getValueAsDouble())) {
            droppedSamples++;
          }
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          if (genericQueues.get(i).offerLatest(genericSignals.get(i).getAsDouble())) {
            droppedSamples++;
          }
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          if (timestampQueues.get(i).offerLatest(timestamp)) {
            droppedSamples++;
          }
        }
      } finally {
        Drive.odometryLock.unlock();
      }

      // every ~1s
      if ((loopCount++ % (int) SwerveConstants.kOdometryFrequency) == 0) {
        Logger.recordOutput("Drive/OdomThread/DroppedSamples", droppedSamples);
      }
    }
  }

  static boolean isValidCanLatency(double latencySeconds) {
    return Double.isFinite(latencySeconds)
        && latencySeconds >= 0.0
        && latencySeconds <= MAX_VALID_CAN_LATENCY_SECONDS;
  }
}
