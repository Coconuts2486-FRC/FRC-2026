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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Elastic;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIParsing;
import frc.robot.util.RBSISubsystem;
import frc.robot.util.TimeUtil;
import frc.robot.util.TimedPose;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Drive subsystem (RBSISubsystem)
 *
 * <p>The Drive subsystem controls the individual swerve Modules and owns the odometry of the robot.
 * The odometry is updated from both the swerve modules and (optionally) the vision subsystem.
 */
public class Drive extends RBSISubsystem {
  private static final SwerveModuleState[] EMPTY_MODULE_STATES = new SwerveModuleState[0];

  // Declare Hardware
  private final Imu imu;
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  // Pose Buffer Declarations
  private final ConcurrentTimeInterpolatableBuffer<Pose2d> poseBuffer =
      ConcurrentTimeInterpolatableBuffer.createBuffer(DrivebaseConstants.kHistorySize);
  private final ConcurrentTimeInterpolatableBuffer<Double> yawBuffer =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(DrivebaseConstants.kHistorySize);
  private final ConcurrentTimeInterpolatableBuffer<Double> yawRateBuffer =
      ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(DrivebaseConstants.kHistorySize);

  // Declare an alert
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final Alert pathPlannerStartPoseAlert =
      new Alert("PathPlanner auto start pose is outside the allowed radius.", AlertType.kError);
  private boolean pathPlannerStartBlocked = false;
  private PathPlannerStartAction pendingPathPlannerStartAction;
  private Pose2d pendingPathPlannerStartPose;

  enum PathPlannerStartAction {
    RESET_TO_PATH_START,
    USE_VISION_POSE,
    BLOCK_AUTO
  }

  // Declare odometry and pose-related variables
  // This one is package-private; used in DriveOdometry, PhoenixOdometryThread, and
  // SparkOdometryThread
  static final Lock odometryLock = new ReentrantLock();
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator m_PoseEstimator =
      new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, lastModulePositions, Pose2d.kZero);

  // Declare PID controller and siumulation physics
  private final ProfiledPIDController angleController;
  private DriveSimPhysics simPhysics;

  private boolean lastOnOpponentHalf = false;
  private static final double FIELD_LENGTH_METERS = 16.54175; // official FRC 2026 field length
  private static final double MIDFIELD_X = /*FIELD_LENGTH_METERS / 3.0*/ 11.821474447275452;

  // Pose reset gate (vision + anything latency-sensitive)
  private volatile long poseResetEpoch = 0; // monotonic counter
  private volatile double lastPoseResetTimestamp = Double.NEGATIVE_INFINITY;
  private volatile double lastAcceptedVisionReceiptTimestamp = Double.NEGATIVE_INFINITY;
  private volatile double lastAcceptedVisionMeasurementTimestamp = Double.NEGATIVE_INFINITY;

  // Pose Regimes (ENABLED, DISABLED_COAST, DISABLE_STATIONARY)
  private boolean lastEnabled = false;
  private double disabledCoastUntilTs = Double.NEGATIVE_INFINITY;
  private double disabledCoastStartTs = Double.NEGATIVE_INFINITY;
  private final double[] lastWheelDistM = new double[4];
  private boolean haveLastWheelDist = false;
  private int stationaryLoops = 0;

  // Related to vision injection of pose
  private boolean disabledVisionInitialized = false;
  private Pose2d lastDisabledVisionPose = new Pose2d();
  private double lastDisabledVisionTs = Double.NaN;
  private final LatestSampleQueue<TimedPose> pendingVisionMeasurements =
      new LatestSampleQueue<>(16);
  private long droppedVisionMeasurements = 0;

  /** Constructor */
  public Drive(Imu imu) {
    this.imu = imu;

    // Define the Angle Controller
    angleController =
        new ProfiledPIDController(
            DrivebaseConstants.kPSPin,
            DrivebaseConstants.kISPin,
            DrivebaseConstants.kDSpin,
            new TrapezoidProfile.Constraints(
                getMaxAngularSpeedRadPerSec(), getMaxAngularAccelRadPerSecPerSec()));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // If REAL (i.e., NOT simulation), parse out the module types
    if (Constants.getMode() == Mode.REAL) {

      // Case out the swerve types because Az-RBSI supports a lot
      switch (Constants.getSwerveType()) {
        case PHOENIX6:
          // This one is easy because it's all CTRE all the time
          for (int i = 0; i < 4; i++) {
            modules[i] = new Module(new ModuleIOTalonFX(i), i);
          }
          break;

        case YAGSL:
          // Then parse the module(s)
          Byte modType = RBSIParsing.parseModuleType();
          for (int i = 0; i < 4; i++) {
            switch (modType) {
              case 0b00000000: // ALL-CTRE
                if (kImuType.equals("navx") || kImuType.equals("navx_spi")) {
                  modules[i] = new Module(new ModuleIOTalonFX(i), i);
                } else {
                  throw new RuntimeException(
                      "For an all-CTRE drive base, use Phoenix Tuner X Swerve Generator instead of YAGSL!");
                }
                break;
              case 0b00010000: // Blended Talon Drive / NEO Steer
                modules[i] = new Module(new ModuleIOBlended(i), i);
                break;
              default:
                throw new RuntimeException("Invalid swerve module combination");
            }
          }
          break;

        default:
          throw new RuntimeException("Invalid Swerve Drive Type");
      }
      // Start odometry thread (for the real robot)

      PhoenixOdometryThread.getInstance().start();

    } else {

      // If SIM, just order up some SIM modules!
      for (int i = 0; i < 4; i++) {
        modules[i] = new Module(new ModuleIOSim(), i);
      }

      // Load the physics simulator
      simPhysics =
          new DriveSimPhysics(
              kinematics,
              RobotConstants.kRobotMOI, // kg m^2
              RobotConstants.kMaxWheelTorque); // Nm
    }

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Configure Autonomous Path Building for PathPlanner based on `AutoType`
    switch (Constants.getAutoType()) {
      case PATHPLANNER:
        try {
          // Configure AutoBuilder for PathPlanner
          AutoBuilder.configure(
              this::getPose,
              this::resetPoseFromPathPlanner,
              this::getChassisSpeeds,
              (speeds, feedforwards) -> runVelocity(speeds),
              new PPHolonomicDriveController(
                  new PIDConstants(
                      DrivebaseConstants.kPStrafe,
                      DrivebaseConstants.kIStrafe,
                      DrivebaseConstants.kDStrafe),
                  new PIDConstants(
                      DrivebaseConstants.kPSPin,
                      DrivebaseConstants.kISPin,
                      DrivebaseConstants.kDSpin)),
              AutoConstants.kPathPlannerConfig,
              () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
              this);
        } catch (Exception e) {
          DriverStation.reportError(
              "Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
              Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
        break;

      case CHOREO:
        // TODO: If your team is using Choreo, you'll know what to do here...
        break;

      case MANUAL:
        // Nothing to be done for MANUAL; may just use AutoPilot
        break;
      default:
    }

    // Configure SysId for drivebase characterization
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  /************************************************************************* */
  /** Periodic function that is called each cycle by the command scheduler */
  @Override
  public void rbsiPeriodic() {

    // The only function of the drive periodic() is to stop the modules if the DriverStation is
    // diabled.
    if (DriverStation.isDisabled()) {
      for (var module : modules) module.stop();
      Logger.recordOutput("SwerveStates/Setpoints", EMPTY_MODULE_STATES);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", EMPTY_MODULE_STATES);
    }

    boolean onOpponentHalf = onOpponentHalf();

    // Only switch tabs when the value CHANGES
    if (onOpponentHalf != lastOnOpponentHalf) {
      Elastic.selectTab(onOpponentHalf ? 2 : 0);
      lastOnOpponentHalf = onOpponentHalf;
    }
  }

  /**
   * Simulation Periodic Method
   *
   * <p>This function runs only for simulation, but does similar processing to the REAL periodic
   * function. Instead of reading back what the modules actually say, use physics to predict where
   * the module would have gone.
   */
  @Override
  public void simulationPeriodic() {

    // IMPORTANT: do not run sim physics during REPLAY
    if (Constants.getMode() != Mode.SIM) return;

    final double dt = Constants.loopPeriodSecs;

    // Advance module wheel physics
    for (int i = 0; i < modules.length; i++) {
      modules[i].simulationPeriodic();
    }

    // Get module states from modules (ok to allocate; can be cached later if desired)
    final SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }

    // Update SIM physics (linear & angular motion of the robot)
    simPhysics.update(moduleStates, dt);

    // Feed the simulated IMU from authoritative physics
    final double yawRad = simPhysics.getYaw().getRadians();
    final double omegaRadPerSec = simPhysics.getOmegaRadPerSec();

    final double ax = simPhysics.getLinearAccel().getX();
    final double ay = simPhysics.getLinearAccel().getY();

    imu.simulationSetYawRad(yawRad);
    imu.simulationSetOmegaRadPerSec(omegaRadPerSec);
    imu.simulationSetLinearAccelMps2(ax, ay, 0.0);

    // Logging ONLY for physics (NOT estimator)
    Logger.recordOutput("Sim/Pose", simPhysics.getPose());
    Logger.recordOutput("Sim/YawRad", yawRad);
    Logger.recordOutput("Sim/OmegaRadPerSec", omegaRadPerSec);
    Logger.recordOutput("Sim/LinearAccelXY_mps2", new double[] {ax, ay});
  }

  /************************************************************************* */
  /** Drive Base Action Functions ****************************************** */

  /**
   * Sets the swerve drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (Module swerveModule : modules) {
      swerveModule.setBrakeMode(brake);
    }
  }

  /** Stop the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    if (pathPlannerStartBlocked && DriverStation.isAutonomousEnabled()) {
      for (Module module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", EMPTY_MODULE_STATES);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", EMPTY_MODULE_STATES);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
      return;
    }

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive in a straight line with the specified drive output
   *
   * @param output Specified drive output for characterization
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /**
   * Reset the heading for the ProfiledPIDController
   *
   * <p>Call this when: (A) robot is disabled, (B) gyro is zeroed, (C) autonomous starts
   */
  public void resetHeadingController() {
    angleController.reset(getHeading().getRadians());
  }

  /**
   * Update the Disabled Coast State
   *
   * <p>The purpose of this function is to determine the coasting state of the robot on the ENABLE
   * -> DISABLE edge. While the robot coasts to a stop, the wheel odometry will continue to
   * integrate with usual vision input. Once the robot stops moving (within tolerance), the vision
   * injection to the Pose will take over.
   *
   * @param enabledNow Are we enabled now?
   * @param disabledNow Are we disabled now?
   * @param now When is now?
   * @param yawRateRadPerSec Current drivebase rotation rate
   * @param odomPositions List of module odometry positions
   */
  public void updateDisabledCoastState(
      boolean enabledNow,
      boolean disabledNow,
      double now,
      double yawRateRadPerSec,
      SwerveModulePosition[] odomPositions) {

    // Don’t end coast “instantly” right after disable edge
    final double minCoastTime = 0.25; // seconds -- maybe put into Constants???
    final boolean pastMin = (now - disabledCoastStartTs) >= minCoastTime;

    // Detect ENABLED -> DISABLED edge -- set `disabledCoastUntilTs` when COAST-phase ends
    if (lastEnabled && !enabledNow) {
      disabledCoastStartTs = now;
      disabledCoastUntilTs = now + DrivebaseConstants.kDisabledCoastSeconds;

      stationaryLoops = 0;
      haveLastWheelDist = false; // reset delta baseline on transition
    }
    lastEnabled = enabledNow;

    // If not disabled, no coast.
    if (!disabledNow) {
      stationaryLoops = 0;
      haveLastWheelDist = false;
      return;
    }

    // If coast already expired, nothing to do.
    if (!(now < disabledCoastUntilTs)) {
      return;
    }

    // Need odometry positions to detect motion
    if (odomPositions == null || odomPositions.length < 4) {
      return;
    }

    // Compute max wheel delta this loop
    double maxDelta = 0.0;
    if (haveLastWheelDist) {
      for (int i = 0; i < 4; i++) {
        double dist = odomPositions[i].distanceMeters;
        double d = Math.abs(dist - lastWheelDistM[i]);
        if (d > maxDelta) maxDelta = d;
      }
    }

    // Update baseline for next loop
    for (int i = 0; i < 4; i++) {
      lastWheelDistM[i] = odomPositions[i].distanceMeters;
    }
    haveLastWheelDist = true;

    // Stationary test (must have baseline)
    if (haveLastWheelDist
        && maxDelta <= DrivebaseConstants.kStationaryMaxWheelDeltaM
        && Math.abs(yawRateRadPerSec) <= DrivebaseConstants.kStationaryMaxYawRateRadPerSec) {
      stationaryLoops++;
    } else {
      stationaryLoops = 0;
    }

    // End coast early if stationary long enough
    if (pastMin && stationaryLoops >= DrivebaseConstants.kStationaryLoopsToEndCoast) {
      disabledCoastUntilTs = now; // expires immediately
    }

    // Debug logs (optional)
    Logger.recordOutput("Odometry/Coast/active", isDisabledCoast(now));
    Logger.recordOutput("Odometry/Coast/untilTs", disabledCoastUntilTs);
    Logger.recordOutput("Odometry/Coast/stationaryLoops", stationaryLoops);
    Logger.recordOutput("Odometry/Coast/maxWheelDeltaM", maxDelta);
    Logger.recordOutput("Odometry/Coast/yawRateRadPerSec", yawRateRadPerSec);
  }

  /************************************************************************* */
  /** SysId Characterization Routines ************************************** */

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /************************************************************************* */
  /** Getter Functions ***************************************************** */

  /** Returns the module array */
  public Module[] getModules() {
    return modules;
  }

  /** Return the prodiledPID angle controller */
  public ProfiledPIDController getAngleController() {
    return angleController;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveStates/MeasuredDoubleArray")
  private double[] getModuleStatesAsDoubleArrayForElastic() {
    return getModuleStatesAsDoubleArray();
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Positions")
  SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the current odometry pose.
   *
   * <p>If the code is running as pure simulation (i.e., not REPLAY of a log), return the simulated
   * physics pose. Otherwise, return the pose from the pose estimator.
   */
  public Pose2d getPose() {
    if (Constants.isPureSim()) {
      return simPhysics.getPose();
    }
    return m_PoseEstimator.getEstimatedPosition();
  }

  public Pose3d get3dPose() {
    return new Pose3d(m_PoseEstimator.getEstimatedPosition());
  }

  /** Returns the current odometry YAW. */
  @AutoLogOutput(key = "Odometry/Yaw")
  public Rotation2d getHeading() {
    if (Constants.isPureSim()) {
      return simPhysics.getYaw();
    }
    return m_PoseEstimator.getEstimatedPosition().getRotation();
  }

  private Rotation2d getRawGyroHeading() {
    if (Constants.isPureSim()) {
      return simPhysics.getYaw();
    }
    return imu.getYaw();
  }

  /**
   * Returns the measured chassis speeds of the modules in FIELD coordinates.
   *
   * <p>+X = field forward +Y = field left CCW+ = counterclockwise
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/FieldMeasured")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    // Robot-relative measured speeds from modules
    ChassisSpeeds robotRelative = getChassisSpeeds();
    // Convert to field-relative using authoritative yaw
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getHeading());
  }

  /**
   * Returns the FIELD-relative linear velocity of the robot's center.
   *
   * <p>+X = field forward +Y = field left
   */
  @AutoLogOutput(key = "Drive/FieldLinearVelocity")
  public Translation2d getFieldLinearVelocity() {
    ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();
    return new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
  }

  /** Returns interpolated odometry pose at a given timestamp. */
  public Optional<Pose2d> getPoseAtTime(double timestampSeconds) {
    return poseBuffer.getSample(timestampSeconds);
  }

  /** Returns the oldest timetamp in the current pose buffer */
  public double getPoseBufferOldestTime() {
    return poseBuffer.getOldestTimestamp().getAsDouble();
  }

  /** Returns the newest timetamp in the current pose buffer */
  public double getPoseBufferNewestTime() {
    return poseBuffer.getNewestTimestamp().getAsDouble();
  }

  /**
   * Max abs yaw rate over [t0, t1] using buffered yaw-rate history
   *
   * @param t0 Interval start
   * @param t1 interval end
   * @return Maximum yaw rate
   */
  public OptionalDouble getMaxAbsYawRateRadPerSec(double t0, double t1) {
    // If end before start, return empty
    if (t1 < t0) return OptionalDouble.empty();

    // Get the subset of entries from the buffer
    var sub = yawRateBuffer.getInternalBuffer().subMap(t0, true, t1, true);
    if (sub.isEmpty()) return OptionalDouble.empty();

    double maxAbs = 0.0;
    boolean any = false;
    for (double v : sub.values()) {
      any = true;
      double a = Math.abs(v);
      if (a > maxAbs) maxAbs = a;
    }
    // Return a value if there's anything to report, else empty
    return any ? OptionalDouble.of(maxAbs) : OptionalDouble.empty();
  }

  /** Get the last EPOCH of a pose reset */
  public long getPoseResetEpoch() {
    return poseResetEpoch;
  }

  /** Get the last TIMESTAMP of a pose reset */
  public double getLastPoseResetTimestamp() {
    return lastPoseResetTimestamp;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DrivebaseConstants.kMaxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / kDriveBaseRadiusMeters;
  }

  /** Returns the maximum linear acceleration in meters per sec per sec. */
  public double getMaxLinearAccelMetersPerSecPerSec() {
    return DrivebaseConstants.kMaxLinearAccel;
  }

  /** Returns the maximum angular acceleration in radians per sec per sec */
  public double getMaxAngularAccelRadPerSecPerSec() {
    return getMaxLinearAccelMetersPerSecPerSec() / kDriveBaseRadiusMeters;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(kFLXPosMeters, kFLYPosMeters),
      new Translation2d(kFRXPosMeters, kFRYPosMeters),
      new Translation2d(kBLXPosMeters, kBLYPosMeters),
      new Translation2d(kBRXPosMeters, kBRYPosMeters)
    };
  }

  /** Returns whether the robot is currently in the DISABLED_COAST state */
  public boolean isDisabledCoast() {
    return isDisabledCoast(TimeUtil.now());
  }

  /** Returns whether the robot was in the DISABLED_COAST state at time `timestamp` */
  public boolean isDisabledCoast(double timestamp) {
    return DriverStation.isDisabled() && (timestamp < disabledCoastUntilTs);
  }

  /** Returns the disabledCoastStartTs variable */
  public double getDisabledCoastStartTs() {
    return disabledCoastStartTs;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /************************************************************************* */
  /* Setter Functions ****************************************************** */

  /**
   * Resets the current odometry pose
   *
   * @param pose The specified pose to which to reset the poseEsitmator
   */
  public void resetPose(Pose2d pose) {
    final double now = TimeUtil.now();
    m_PoseEstimator.resetPosition(getRawGyroHeading(), getModulePositions(), pose);
    lastAcceptedVisionReceiptTimestamp = Double.NEGATIVE_INFINITY;
    markPoseReset(now);
    poseBufferAddSample(now, pose);
  }

  /** Validates PathPlanner's starting pose and returns whether autonomous may run. */
  public boolean validatePathPlannerAutoStart(Pose2d pathStartPose) {
    pendingPathPlannerStartAction = evaluatePathPlannerStart(pathStartPose);
    pendingPathPlannerStartPose = pathStartPose;
    return pendingPathPlannerStartAction != PathPlannerStartAction.BLOCK_AUTO;
  }

  private PathPlannerStartAction evaluatePathPlannerStart(Pose2d pathStartPose) {
    final double now = TimeUtil.now();
    final double visionAge = now - lastAcceptedVisionReceiptTimestamp;
    final boolean hasRecentVision =
        Double.isFinite(visionAge)
            && visionAge >= 0.0
            && visionAge <= DrivebaseConstants.kPathPlannerVisionFreshnessSec;
    final Pose2d currentPose = getPose();
    final double startDistanceMeters =
        currentPose.getTranslation().getDistance(pathStartPose.getTranslation());
    final PathPlannerStartAction action =
        determinePathPlannerStartAction(
            currentPose,
            pathStartPose,
            hasRecentVision,
            DrivebaseConstants.kPathPlannerStartToleranceMeters);

    pathPlannerStartBlocked = action == PathPlannerStartAction.BLOCK_AUTO;
    pathPlannerStartPoseAlert.setText(
        String.format(
            "PathPlanner auto blocked: robot is %.2f m from the requested start (limit %.2f m).",
            startDistanceMeters, DrivebaseConstants.kPathPlannerStartToleranceMeters));
    pathPlannerStartPoseAlert.set(pathPlannerStartBlocked);

    Logger.recordOutput("Auto/NominalStartingPose", pathStartPose);
    Logger.recordOutput("Auto/PoseBeforeResetDecision", currentPose);
    Logger.recordOutput("Auto/VisionMeasurementAgeSec", visionAge);
    Logger.recordOutput(
        "Auto/LastVisionMeasurementTimestamp", lastAcceptedVisionMeasurementTimestamp);
    Logger.recordOutput("Auto/StartPoseDistanceMeters", startDistanceMeters);
    Logger.recordOutput("Auto/StartPoseAction", action.toString());
    Logger.recordOutput("Auto/StartPoseBlocked", pathPlannerStartBlocked);
    Logger.recordOutput(
        "Auto/PoseResetSkippedForVision", action == PathPlannerStartAction.USE_VISION_POSE);

    return action;
  }

  static PathPlannerStartAction determinePathPlannerStartAction(
      Pose2d currentPose, Pose2d pathStartPose, boolean hasRecentVision, double toleranceMeters) {
    boolean estimatorUninitialized = currentPose.getTranslation().getNorm() <= 1e-6;
    if (!hasRecentVision || estimatorUninitialized) {
      return PathPlannerStartAction.RESET_TO_PATH_START;
    }

    double distance = currentPose.getTranslation().getDistance(pathStartPose.getTranslation());
    return Double.isFinite(distance) && distance <= toleranceMeters
        ? PathPlannerStartAction.USE_VISION_POSE
        : PathPlannerStartAction.BLOCK_AUTO;
  }

  /** Applies PathPlanner's starting-pose policy from AutoBuilder's reset callback. */
  private void resetPoseFromPathPlanner(Pose2d pose) {
    PathPlannerStartAction action;
    boolean usingPreflightDecision =
        pendingPathPlannerStartAction != null
            && pendingPathPlannerStartPose != null
            && pendingPathPlannerStartPose.getTranslation().getDistance(pose.getTranslation())
                <= 1e-6
            && Math.abs(
                    pendingPathPlannerStartPose
                        .getRotation()
                        .minus(pose.getRotation())
                        .getRadians())
                <= 1e-6;

    if (usingPreflightDecision) {
      action = pendingPathPlannerStartAction;
    } else {
      action = evaluatePathPlannerStart(pose);
    }
    pendingPathPlannerStartAction = null;
    pendingPathPlannerStartPose = null;

    boolean resetSuppressed = action != PathPlannerStartAction.RESET_TO_PATH_START;
    Logger.recordOutput("Auto/PathPlannerResetUsedPreflightDecision", usingPreflightDecision);
    Logger.recordOutput("Auto/PathPlannerResetSuppressed", resetSuppressed);

    if (!resetSuppressed) {
      resetPose(pose);
    }
  }

  /** Zeros the gyro based on alliance color */
  public void zeroHeadingForAlliance() {
    imu.zeroYaw(
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Rotation2d.kZero
            : Rotation2d.k180deg);
    resetHeadingController();
    markPoseReset(TimeUtil.now());
  }

  /** Zeros the gyro regardless of the alliance */
  public void zeroHeading() {
    imu.zeroYaw(Rotation2d.kZero);
    resetHeadingController();
    markPoseReset(TimeUtil.now());
  }

  /**
   * Adds a vision measurement safely into the PoseEstimator
   *
   * @param measurement The pose @ timestamp to add to the pose estimator
   */
  // Called by Vision via consumer.accept(TimedPose). Odometry applies queued measurements while it
  // already owns the estimator lock.
  public void addVisionMeasurement(TimedPose meas) {
    if (pendingVisionMeasurements.offerLatest(meas)) {
      droppedVisionMeasurements++;
    }
  }

  /** Applies a bounded number of queued vision measurements. Caller must own odometryLock. */
  int applyQueuedVisionMeasurements(int maxMeasurements) {
    int processed = 0;
    while (processed < maxMeasurements) {
      TimedPose measurement = pendingVisionMeasurements.poll();
      if (measurement == null) {
        break;
      }
      applyVisionMeasurementLocked(measurement);
      processed++;
    }

    Logger.recordOutput("Vision/EstimatorQueueDepth", pendingVisionMeasurements.size());
    Logger.recordOutput("Vision/EstimatorQueueDropped", droppedVisionMeasurements);
    Logger.recordOutput("Vision/EstimatorQueueProcessed", processed);
    return processed;
  }

  private void applyVisionMeasurementLocked(TimedPose meas) {
    // Always use measurement timestamp when fusing (enabled path)
    final double t = meas.timestampSeconds();
    final Pose2d vision = meas.pose();

    // ENABLED: normal fusion
    if (!DriverStation.isDisabled()) {
      disabledVisionInitialized = false;
      lastDisabledVisionTs = Double.NaN;
      m_PoseEstimator.addVisionMeasurement(vision, t, meas.stdDevs());
      markVisionMeasurementAccepted(t);
      return;
    }

    // DISABLED -- check if within "coast phase"
    final boolean coast = isDisabledCoast(t);

    // If coasting,
    if (coast) {
      final double coastAge = t - getDisabledCoastStartTs();
      // Logger.recordOutput("Vision/Debug/disabledCoastAge", coastAge);

      // Ignore vision briefly right after ENABLE->DISABLE (prevents “phase mismatch” at disable
      // edge)
      if (coastAge >= 0.0 && coastAge < DrivebaseConstants.kDisabledVisionIgnoreAfterDisableSec) {
        // Logger.recordOutput("Vision/Debug/disabledIgnoreEarlyCoast", true);
        return;
      }
    }
    // Logger.recordOutput("Vision/Debug/disabledIgnoreEarlyCoast", false);

    // If we're coasting, avoid snapping Pose to Vision; lean gentler than stationary.
    final double alpha =
        coast
            ? Math.min(DrivebaseConstants.kDisabledVisionBlendAlpha, 0.05)
            : DrivebaseConstants.kDisabledVisionBlendAlpha;

    // Debug
    // Logger.recordOutput("Vision/Debug/disabledCoast", coast);
    // Logger.recordOutput("Vision/Debug/disabledVisionInitialized", disabledVisionInitialized);
    // Logger.recordOutput("Vision/Debug/disabledVisionTs", t);
    // Logger.recordOutput(
    //     "Vision/Debug/disabledVisionAge",
    //     Double.isFinite(lastDisabledVisionTs) ? (t - lastDisabledVisionTs) : Double.NaN);

    // Check if the last while-disabled vision timestamp is stale (too old)
    final boolean stale =
        Double.isFinite(lastDisabledVisionTs)
            && (t - lastDisabledVisionTs) > DrivebaseConstants.kDisabledVisionStale;
    // Logger.recordOutput("Vision/Debug/visionStale", stale);

    // If coasting, intentionally DO NOT snap; reset initialization so that once coast ends, the
    // first good stationary frame snaps.
    if (coast) {
      disabledVisionInitialized = false;
    }

    // If not initialized AND not coasting: snap hard to vision once
    if (!disabledVisionInitialized && !coast) {
      disabledVisionInitialized = true;
      lastDisabledVisionPose = vision;
      lastDisabledVisionTs = t;

      m_PoseEstimator.resetPosition(getRawGyroHeading(), getModulePositions(), vision);
      markPoseReset(t);
      poseBufferAddSample(t, vision);
      markVisionMeasurementAccepted(t);

      Logger.recordOutput("Vision/DisabledInitSnap", true);
      Logger.recordOutput("Vision/DisabledReject", false);
      Logger.recordOutput("Vision/DisabledBlendAlphaUsed", alpha);
      return;
    }
    Logger.recordOutput("Vision/DisabledInitSnap", false);

    // Check that there is not a huge jump from the last accepted disabled vision pose
    final Pose2d gateRef = Double.isFinite(lastDisabledVisionTs) ? lastDisabledVisionPose : vision;

    final double deltaTranslation = gateRef.getTranslation().getDistance(vision.getTranslation());
    final double deltaRotation =
        Math.abs(gateRef.getRotation().minus(vision.getRotation()).getRadians());

    // Logger.recordOutput("Vision/Debug/dTransFromLastVision", deltaTranslation);
    // Logger.recordOutput("Vision/Debug/dRotFromLastVision", deltaRotation);

    // Reject large jumps only if vision measurement is not stale (large delta-T can mean large
    // change in position)
    if (!stale
        && (deltaTranslation > DrivebaseConstants.kDisabledVisionMaxJumpM
            || deltaRotation > DrivebaseConstants.kDisabledVisionMaxJumpRad)) {
      Logger.recordOutput("Vision/DisabledReject", true);
      Logger.recordOutput("Vision/DisabledBlendAlphaUsed", alpha);
      return;
    }
    Logger.recordOutput("Vision/DisabledReject", false);

    // Accept this vision frame as the new reference
    lastDisabledVisionPose = vision;
    lastDisabledVisionTs = t;

    // After the one initialization snap, use the estimator's timestamped vision update. Repeated
    // resetPosition calls invalidate the pose history and force Vision to clear its smoothing
    // state every frame.
    m_PoseEstimator.addVisionMeasurement(vision, t, meas.stdDevs());
    final Pose2d fusedPose = m_PoseEstimator.getEstimatedPosition();
    poseBufferAddSample(t, fusedPose);
    markVisionMeasurementAccepted(t);

    Logger.recordOutput("Vision/DisabledBlendedPose", fusedPose);
    Logger.recordOutput("Vision/DisabledBlendAlphaUsed", 0.0);
  }

  private void markVisionMeasurementAccepted(double measurementTimestamp) {
    lastAcceptedVisionMeasurementTimestamp = measurementTimestamp;
    lastAcceptedVisionReceiptTimestamp = TimeUtil.now();
  }

  /**
   * Sets the EPOCH and TIMESTAMP for a pose reset
   *
   * @param fpgaNow The FPGA timestamp of the pose reset
   */
  private void markPoseReset(double fpgaNow) {
    lastPoseResetTimestamp = fpgaNow;
    poseResetEpoch++;
    Logger.recordOutput("Drive/PoseResetEpoch", poseResetEpoch);
    Logger.recordOutput("Drive/PoseResetTimestamp", lastPoseResetTimestamp);
  }

  /************************************************************************* */
  /**
   * DriveOdometry Helpers (package-private)
   *
   * <p>The pose estimator and pose buffers are owned by Drive, but DriveOdometry needs access to
   * them in order to update and process the odometry. These functions are the appropriate
   * pass-throughs to allow this functionality.
   */

  /** Update the pose estimator at a timestamp */
  void poseEstimatorUpdateWithTime(double t, Rotation2d yaw, SwerveModulePosition[] positions) {
    m_PoseEstimator.updateWithTime(t, yaw, positions);
  }

  /** Add a sample to the pose buffer */
  void poseBufferAddSample(double t, Pose2d pose) {
    poseBuffer.addSample(t, pose);
  }

  /** Yaw buffer helper */
  double yawBufferSampleOr(double t, double fallbackYawRad) {
    return yawBuffer.getSample(t).orElse(fallbackYawRad);
  }

  /** Yaw buffer helper */
  void yawBuffersAddSample(double t, double yawRad, double yawRateRadPerSec) {
    yawBuffer.addSample(t, yawRad);
    yawRateBuffer.addSample(t, yawRateRadPerSec);
  }

  /** Yaw buffer helper */
  void yawBuffersFillFromQueue(double[] yawTs, double[] yawPosRad) {
    for (int k = 0; k < yawTs.length; k++) {
      yawBuffer.addSample(yawTs[k], yawPosRad[k]);
      if (k > 0) {
        double dt = yawTs[k] - yawTs[k - 1];
        if (dt > 1e-6) {
          yawRateBuffer.addSample(yawTs[k], calculateYawRate(yawPosRad[k - 1], yawPosRad[k], dt));
        }
      }
    }
  }

  /** Yaw buffer helper */
  void yawBuffersAddSampleIndexAligned(double t, double[] yawTs, double[] yawPos, int i) {
    yawBuffer.addSample(t, yawPos[i]);
    if (i > 0) {
      double dt = yawTs[i] - yawTs[i - 1];
      if (dt > 1e-6) {
        yawRateBuffer.addSample(t, calculateYawRate(yawPos[i - 1], yawPos[i], dt));
      }
    }
  }

  static double calculateYawRate(double previousYawRad, double yawRad, double dtSeconds) {
    return MathUtil.angleModulus(yawRad - previousYawRad) / dtSeconds;
  }

  /** Set the gyroDisconnectedAlert */
  void setGyroDisconnectedAlert(boolean disconnected) {
    gyroDisconnectedAlert.set(disconnected);
  }

  /************************************************************************* */
  /** Simulation Getter Functions (from simPhysics) */
  public Pose2d getSimPose() {
    return simPhysics.getPose();
  }

  public double getSimYawRad() {
    return simPhysics.getYaw().getRadians();
  }

  public double getSimYawRateRadPerSec() {
    return simPhysics.getOmegaRadPerSec();
  }

  /************************************************************************* */
  /** CHOREO SECTION (Ignore if AutoType == PATHPLANNER) ******************* */

  /** Choreo: Reset odometry */
  public void resetOdometry(Pose2d pose) {
    resetPose(pose);
  }

  private double[] getModuleStatesAsDoubleArray() {
    SwerveModuleState[] states = getModuleStates();
    return new double[] {
      states[0].angle.getDegrees(),
      states[0].speedMetersPerSecond,
      states[1].angle.getDegrees(),
      states[1].speedMetersPerSecond,
      states[2].angle.getDegrees(),
      states[2].speedMetersPerSecond,
      states[3].angle.getDegrees(),
      states[3].speedMetersPerSecond
    };
  }

  public boolean onOpponentHalf() {
    Pose2d pose = m_PoseEstimator.getEstimatedPosition(); // or getPose() if you already have it

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    double allianceRelativeX = isRed ? (FIELD_LENGTH_METERS - pose.getX()) : pose.getX();

    return allianceRelativeX > MIDFIELD_X;
  }

  /** Dummy function to make things happy -- doesn't actually do anything */
  @Override
  public int[] getPowerPorts() {
    return new int[] {};
  }
}
