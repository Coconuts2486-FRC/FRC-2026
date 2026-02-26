// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANBuses;
import frc.robot.Constants.Cameras;
import frc.robot.Constants.OperatorConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.driver_info.CANStatus;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.imu.ImuIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.prematch.Prematch;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.CameraSweepEvaluator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GetJoystickValue;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.RBSICANBusRegistry;
import frc.robot.util.RBSICANHealth;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.DriveStyle;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIPowerMonitor;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {

  /** Define the Driver and, optionally, the Operator/Co-Driver Controllers */
  // Replace with ``CommandPS4Controller`` or ``CommandJoystick`` if needed
  final CommandXboxController driverController = new CommandXboxController(0); // Main Driver

  final CommandXboxController operatorController = new CommandXboxController(1); // Second Operator
  final OverrideSwitches overrides = new OverrideSwitches(2); // Console toggle switches

  // These two are needed for the Sweep evaluator for camera FOV simulation
  final CommandJoystick joystick3 = new CommandJoystick(3); //  Joystick for CamersSweepEvaluator
  private final CameraSweepEvaluator sweep;

  /** Declare the robot subsystems here ************************************ */
  // These are the "Active Subsystems" that the robot controls
  private final Drive m_drivebase;

  // private final Flywheel m_flywheel;
  private final Intake m_intake;
  private final Climb m_climb;
  private final Indexer m_indexer;
  private final Feeder m_feeder;
  private final Shooter m_shooter;

  private boolean elasticOnDriveTab = true;
  private final Turret m_turret;
  private final Prematch m_prematch;

  // ... Add additional subsystems here (e.g., elevator, arm, etc.)

  // These are "Virtual Subsystems" that report information but have no motors
  private final Imu m_imu;

  @SuppressWarnings("unused")
  private final Accelerometer m_accel;

  @SuppressWarnings("unused")
  private final RBSIPowerMonitor m_power;

  @SuppressWarnings("unused")
  private final Vision m_vision;

  @SuppressWarnings("unused")
  private List<RBSICANHealth> m_canHealth;

  @SuppressWarnings("unused")
  private final CANStatus m_canStatus;

  /** Dashboard inputs ***************************************************** */
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;

  private final LoggedDashboardChooser<DriveStyle> driveStyle =
      new LoggedDashboardChooser<>("Drive Style");

  private final AutoChooser autoChooserChoreo;
  private final AutoFactory autoFactoryChoreo;
  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);
  // EXAMPLE TUNABLE FLYWHEEL SPEED INPUT FROM DASHBOARD
  private final LoggedTunableNumber flywheelSpeedInput =
      new LoggedTunableNumber("Flywheel Speed", 1500.0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  // Vision Factories
  private VisionIO[] buildVisionIOsReal(Drive drive) {
    return switch (Constants.getVisionType()) {
      case PHOTON ->
          Arrays.stream(Cameras.ALL)
              .map(c -> (VisionIO) new VisionIOPhotonVision(c.name(), c.robotToCamera()))
              .toArray(VisionIO[]::new);

      case LIMELIGHT ->
          Arrays.stream(Cameras.ALL)
              .map(c -> (VisionIO) new VisionIOLimelight(c.name(), drive::getHeading))
              .toArray(VisionIO[]::new);

      case NONE -> new VisionIO[] {new VisionIO() {}};
    };
  }

  private static VisionIO[] buildVisionIOsSim(Drive drive) {
    var cams = Constants.Cameras.ALL;
    VisionIO[] ios = new VisionIO[cams.length];
    for (int i = 0; i < cams.length; i++) {
      var cfg = cams[i];
      ios[i] = new VisionIOPhotonVisionSim(cfg.name(), cfg.robotToCamera(), drive::getPose);
    }
    return ios;
  }

  private VisionIO[] buildVisionIOsReplay() {
    return new VisionIO[] {new VisionIO() {}};
  }

  /**
   * Constructor for the Robot Container. This container holds subsystems, opertator interface
   * devices, and commands.
   */
  public RobotContainer() {

    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // Register the CANBus
        RBSICANBusRegistry.initReal(CANBuses.RIO, CANBuses.DRIVE);

        // YAGSL drivebase, get config from deploy directory
        // Get the IMU instance
        m_imu = new Imu(SwerveConstants.kImu.factory.get());

        m_drivebase = new Drive(m_imu);
        // m_flywheel = new Flywheel(new FlywheelIOSim()); // new Flywheel(new FlywheelIOTalonFX());
        m_vision = new Vision(m_drivebase::addVisionMeasurement, buildVisionIOsReal(m_drivebase));
        m_accel = new Accelerometer(m_imu);
        m_climb = new Climb(new ClimbIOTalonFX());
        m_intake = new Intake(new IntakeIOTalonFX());
        m_indexer = new Indexer(new IndexerIOTalonFX());
        m_feeder = new Feeder(new FeederIOTalonFX());
        m_shooter = new Shooter(new ShooterIOTalonFX());
        m_turret = new Turret(new TurretIOTalonFX());
        m_prematch = new Prematch(m_turret, m_intake);
        sweep = null;

        break;

      case SIM:
        RBSICANBusRegistry.initSim(CANBuses.RIO, CANBuses.DRIVE);

        m_imu = new Imu(new ImuIOSim());
        m_drivebase = new Drive(m_imu);
        // m_flywheel = new Flywheel(new FlywheelIOSim() {});
        m_vision = new Vision(m_drivebase::addVisionMeasurement, buildVisionIOsSim(m_drivebase));
        m_accel = new Accelerometer(m_imu);
        m_climb = new Climb(new ClimbIOSim());
        m_intake = new Intake(new IntakeIOSim());
        m_indexer = new Indexer(new IndexerIOSim());
        m_feeder = new Feeder(new FeederIOSim());
        m_shooter = new Shooter(new ShooterIOSim());
        m_turret = new Turret(new TurretIOSim());
        m_prematch = null;

        // ---------------- CameraSweepEvaluator (sim-only analysis) ----------------
        var cams = Cameras.ALL;
        VisionSystemSim visionSim = new VisionSystemSim("CameraSweepWorld");
        visionSim.addAprilTags(FieldConstants.aprilTagLayout);

        PhotonCameraSim[] simCams = new PhotonCameraSim[cams.length];

        for (int i = 0; i < cams.length; i++) {
          var cfg = cams[i];

          PhotonCamera photonCam = new PhotonCamera(cfg.name());
          PhotonCameraSim camSim = new PhotonCameraSim(photonCam, cfg.simProps());

          visionSim.addCamera(camSim, cfg.robotToCamera());
          simCams[i] = camSim;
        }

        // Create the sweep evaluator (expects two cameras; adapt if you add more later)
        if (simCams.length >= 2) {
          sweep = new CameraSweepEvaluator(visionSim, simCams[0], simCams[1]);
        } else {
          sweep = null; // or throw if you require exactly 2 cameras
        }

        break;

      default:
        // Replayed robot, disable IO implementations
        RBSICANBusRegistry.initSim(CANBuses.RIO, CANBuses.DRIVE);
        m_imu = new Imu(new ImuIOSim() {});
        m_drivebase = new Drive(m_imu);
        // m_flywheel = new Flywheel(new FlywheelIO() {});
        m_vision = new Vision(m_drivebase::addVisionMeasurement, buildVisionIOsReplay());
        m_accel = new Accelerometer(m_imu);
        sweep = null;
        m_intake = new Intake(new IntakeIO() {});
        m_indexer = new Indexer(new IndexerIO() {});
        m_feeder = new Feeder(new FeederIO() {});
        m_shooter = new Shooter(new ShooterIO() {});
        m_climb = new Climb(new ClimbIO() {});
        m_turret = new Turret(new TurretIO() {});
        m_prematch = new Prematch(m_turret, m_intake);

        break;
    }

    // Init all CAN busses specified in the `Constants.CANBuses` class
    RBSICANBusRegistry.initReal(Constants.CANBuses.ALL);
    m_canHealth = Arrays.stream(Constants.CANBuses.ALL).map(RBSICANHealth::new).toList();
    m_canStatus = new CANStatus(m_drivebase, m_imu);

    // In addition to the initial battery capacity from the Dashbaord, ``RBSIPowerMonitor`` takes
    // all the non-drivebase subsystems for which you wish to have power monitoring; DO NOT
    // include ``m_drivebase``, as that is automatically monitored.
    m_power = new RBSIPowerMonitor(batteryCapacity);

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case MANUAL:
        // This is where the "Leave Auto" will go
        // ...
        // Set the others to null
        autoChooserPathPlanner = null;
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        break;

      case PATHPLANNER:
        autoChooserPathPlanner =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set the others to null
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        break;

      case CHOREO:
        autoFactoryChoreo =
            new AutoFactory(
                m_drivebase::getPose, // A function that returns the current robot pose
                m_drivebase::resetOdometry, // A function that resets the current robot pose to the
                // provided Pose2d
                m_drivebase::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                m_drivebase // The drive subsystem
                );
        autoChooserChoreo = new AutoChooser();
        autoChooserChoreo.addRoutine("twoPieceAuto", this::twoPieceAuto);
        // Set the others to null
        autoChooserPathPlanner = null;
        break;

      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

    // Get drive style from the Dashboard Chooser
    driveStyle.addDefaultOption("TANK", DriveStyle.TANK);
    driveStyle.addOption("GAMER", DriveStyle.GAMER);

    // Define Auto commands
    defineAutoCommands();
    // Define SysIs Routines
    definesysIdRoutines();
    // Configure the button and trigger bindings
    configureBindings();
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner / Choreo */
  private void defineAutoCommands() {

    // TODO Change the velocity
    NamedCommands.registerCommand("IntakeDown", Commands.runOnce(() -> m_intake.pivotDown()));

    NamedCommands.registerCommand("Intake", Commands.run(() -> m_intake.setRollerVelocity(0.65)));

    // NamedCommands.registerCommand(
    //   "ClimbPrepare",
    //   Commands.runOnce(()));

    // NamedCommands.registerCommand(
    //   "Climb",
    //   Commands.runOnce(()));

    // NamedCommands.registerCommand(
    //   "Shoot",
    //   Commands.runOnce(()));

    // NamedCommands.registerCommand(
    //   "Pass",
    //   Commands.runOnce(()));

    // NamedCommands.registerCommand(
    //   "ClimbPrepare",
    //   Commands.runOnce(()));

    // NamedCommands.registerCommand(
    //   "Climb",
    //   Commands.runOnce(()));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    // Send the proper joystick input based on driver preference -- Set this in `Constants.java`
    GetJoystickValue driveStickY;
    GetJoystickValue driveStickX;
    GetJoystickValue turnStickX;
    // OPTIONAL: Use the DashboardChooser rather than the Constants file for Drive Style
    // switch (driveStyle.get()) {
    switch (OperatorConstants.kDriveStyle) {
      case GAMER:
        driveStickY = driverController::getRightY;
        driveStickX = driverController::getRightX;
        turnStickX = driverController::getLeftX;
        break;
      default: // Includes case TANK
        driveStickY = driverController::getLeftY;
        driveStickX = driverController::getLeftX;
        turnStickX = driverController::getRightX;
    }

    // =======================================================================
    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase,
            () -> -driveStickY.value(),
            () -> -driveStickX.value(),
            () -> -turnStickX.value()));

    // =======================================================================
    // Set DEFAULT COMMANDS for subsystems
    m_indexer.setDefaultCommand(
        // Get the running states of intake rollers & feeder.  Set the indexer to run if
        // either input is true.
        Commands.run(
            () -> {
              if (m_intake.isIntakeRunning() || m_feeder.isFeederRunning()) {
                m_indexer.setVelocity(0.5); // TODO:change this velocity
              } else {
                m_indexer.stop();
              }
            },
            m_indexer));

    m_feeder.setDefaultCommand(
        // Listen to the shooter, and turn on when the shooter is up to speed.
        Commands.run(
            () -> {
              if (
              /*m_shooter.isShooterRunning()*/ m_feeder
                  .isFeederRunning()) { // TODO:change this to shooter when shootering
                m_feeder.runFeeder();
              } else m_feeder.stopFeeder();
            }));

    m_shooter.setDefaultCommand(
        Commands.run(
            () -> {
              // Listen to the Coordinator, and set the velocity based on physics computation.  If
              // in "Don't Shoot" status, set speed to some idle value.
            }));

    m_turret.setDefaultCommand(
        Commands.run(
            () -> {
              // Listen to the Coordinator, and set the angle based on the physics computation.
            }));

    // ===============================================================================
    // ** Example Commands -- Remap, remove, or change as desired **
    // Press B button while driving --> ROBOT-CENTRIC
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () ->
                    DriveCommands.robotRelativeDrive(
                        m_drivebase,
                        () -> -driveStickY.value(),
                        () -> -driveStickX.value(),
                        () -> turnStickX.value())));

    driverController.y().onTrue(Commands.runOnce(() -> m_prematch.enableUpdate()));
    // Press A button -> BRAKE
    // driverController
    //     .a()
    //     .whileTrue(Commands.runOnce(() -> m_drivebase.setMotorBrake(true), m_drivebase));

    // Press X button --> Stop with wheels in X-Lock position
    driverController.x().onTrue(Commands.runOnce(m_drivebase::stopWithX, m_drivebase));
    // Press start button --> switch elastic tab
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  elasticOnDriveTab = !elasticOnDriveTab;
                  Elastic.selectTab(elasticOnDriveTab ? 2 : 0);
                }));

    // Press Start button --> Manually Re-Zero the Gyro
    driverController
        .start()
        .onTrue(
            Commands.runOnce(m_drivebase::zeroHeadingForAlliance, m_drivebase)
                .ignoringDisable(true));

    /*there is a default command in intake that makes it go up this toggles a new command
    that cancels the default and keeps the intake down with out the driver having to hold any button */
    driverController.leftBumper().toggleOnTrue(Commands.run(() -> m_intake.pivotDown()));

    driverController
        .rightBumper()
        .toggleOnTrue(
            Commands.run(() -> m_intake.runRollers())
                .andThen(Commands.runOnce(() -> m_intake.stopRollers())));

    // Testing functions
    driverController.leftBumper().whileTrue(Commands.run(() -> m_indexer.setVelocity(.5)));
    
    driverController.povUp().whileTrue(Commands.run(() -> m_intake.setPivotVelocity(0.05)));
                //reads out pivot position- useful for determining where exactly we need to tell it to pivot to (pov left)
    driverController.povLeft().whileTrue(Commands.run(() -> System.out.println(m_intake.getPosition())));
                //checks magnetic switch (pov right)
    driverController.povRight().whileTrue(Commands.run(() -> System.out.println("Magnetic switch state:"+m_turret.readTurretSwitch())));

    // prints the encoder position temporary testing function
    driverController
        .a()
        .whileTrue(Commands.run(() -> System.out.println(m_turret.readTurretSwitch())));

    // Press LEFT BUMPER --> Drive to a pose 10 feet closer to the BLUE ALLIANCE wall
    // driverController
    //     .leftTrigger()
    //     .whileTrue(
    //         Commands.defer(
    //             () -> {
    //               // New pose 2 feet closer to BLUE ALLIANCE wall
    //               Pose2d pose =
    //                   m_drivebase
    //                       .getPose()
    //                       .transformBy(
    //                           new Transform2d(Units.feetToMeters(-20.0), 0.0, Rotation2d.kZero));

    // Alternatively, you could define a pose in a separate module and call it here.
    //
    // Example from 2025 Reefscape:
    // --------
    // pose = ReefPoses.kBluePoleE;

    //   return AutopilotCommands.runAutopilot(m_drivebase, pose);
    // },
    // Set.of(m_drivebase)));

    // Press POV buttons to nudge the robot in each direction
    driverController
        .povLeft()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(0.), Units.inchesToMeters(11.0), 0.));
                },
                // Stop when command ended
                m_drivebase::stop,
                m_drivebase));

    driverController
        .povRight()
        .whileTrue(
            Commands.run(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(0.), Units.inchesToMeters(-11.0), 0.));
                },
                m_drivebase));

    driverController
        .povUp()
        .whileTrue(
            Commands.run(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(-11), Units.inchesToMeters(0), 0));
                },
                m_drivebase));

    driverController
        .povDown()
        .whileTrue(
            Commands.run(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(11), Units.inchesToMeters(0), 0));
                },
                m_drivebase));

    if (Constants.getMode() == Mode.SIM) {
      // IN SIMULATION ONLY:
      // Double-press the A button on Joystick3 to run the CameraSweepEvaluator
      // Use WPILib's built-in double-press binding
      joystick3
          .button(1)
          .multiPress(2, 0.2)
          .onTrue(
              Commands.runOnce(
                  () -> {
                    try {
                      sweep.runFullSweep(
                          Filesystem.getOperatingDirectory()
                              .toPath()
                              .resolve("camera_sweep.csv")
                              .toString());
                    } catch (Exception e) {
                      e.printStackTrace();
                    }
                  }));
    }
  }

  /**
   * Use this to pass the MANUAL SHOOT FUEL command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getManualAuto() {
    // NOTE:
    //
    // For teams not using PathPlanner, this auto may be used to simply shoot the pre-loaded fuel
    // into the HUB during AUTO.  Since shooters are beyond the scope of Az-RBSI, you will have to
    // write your own command and call it here.

    // Replace Commands.none() with your command that shoots fuel into the HUB.
    return Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandPathPlanner() {
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooserPathPlanner.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommandChoreo() {
    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooserChoreo);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooserChoreo.selectedCommandScheduler());
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = Constants.getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use ("
              + Constants.getAprilTagLayoutType().toString()
              + ").");
    }
  }

  /** Drivetrain getter method for use with Robot.java */
  public Drive getDrivebase() {
    return m_drivebase;
  }

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == AutoType.PATHPLANNER) {
      // Drivebase characterization
      autoChooserPathPlanner.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Forward)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Reverse)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Forward)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Reverse)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      // Example Flywheel SysId Characterization
      // autoChooserPathPlanner.addOption(
      //     "Flywheel SysId (Quasistatic Forward)",
      //     m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // autoChooserPathPlanner.addOption(
      //     "Flywheel SysId (Quasistatic Reverse)",
      //     m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // autoChooserPathPlanner.addOption(
      //     "Flywheel SysId (Dynamic Forward)",
      //     m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // autoChooserPathPlanner.addOption(
      //     "Flywheel SysId (Dynamic Reverse)",
      //     m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  /**
   * Example Choreo auto command
   *
   * <p>NOTE: This would normally be in a spearate file.
   */
  private AutoRoutine twoPieceAuto() {
    AutoRoutine routine = autoFactoryChoreo.newRoutine("twoPieceAuto");

    // Load the routine's trajectories
    AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
    AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(pickupTraj.resetOdometry(), pickupTraj.cmd()));

    // Starting at the event marker named "intake", run the intake
    // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());

    // When the trajectory is done, start the next trajectory
    pickupTraj.done().onTrue(scoreTraj.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return routine;
  }
}
