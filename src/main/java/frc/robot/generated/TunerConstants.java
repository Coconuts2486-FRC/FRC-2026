package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;

/**
 * Dummy class that hold just the variable types; everything is actually instantiated in one of the
 * named classes for one of the robots.
 */
public class TunerConstants {
  private static final Slot0Configs steerGains = null;
  private static final Slot0Configs driveGains = null;
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = null;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = null;
  private static final DriveMotorArrangement kDriveMotorType = null;
  private static final SteerMotorArrangement kSteerMotorType = null;
  private static final SteerFeedbackType kSteerFeedbackType = null;
  private static final Current kSlipCurrent = null;
  private static final TalonFXConfiguration driveInitialConfigs = null;
  private static final TalonFXConfiguration steerInitialConfigs = null;
  private static final CANcoderConfiguration encoderInitialConfigs = null;
  private static final Pigeon2Configuration pigeonConfigs = null;
  public static final CANBus kCANBus = null;
  public static final LinearVelocity kSpeedAt12Volts = null;
  private static final double kCoupleRatio = 0.0;
  private static final double kDriveGearRatio = 0.0;
  private static final double kSteerGearRatio = 0.0;
  private static final Distance kWheelRadius = null;
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = false;
  private static final int kPigeonId = 0;
  private static final MomentOfInertia kSteerInertia = null;
  private static final MomentOfInertia kDriveInertia = null;
  private static final Voltage kSteerFrictionVoltage = null;
  private static final Voltage kDriveFrictionVoltage = null;
  public static final SwerveDrivetrainConstants DrivetrainConstants = null;
  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator = null;
  // Front Left
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 3;
  private static final Angle kFrontLeftEncoderOffset = null;
  private static final boolean kFrontLeftSteerMotorInverted = false;
  private static final boolean kFrontLeftEncoderInverted = false;
  private static final Distance kFrontLeftXPos = null;
  private static final Distance kFrontLeftYPos = null;
  // Front Right
  private static final int kFrontRightDriveMotorId = 4;
  private static final int kFrontRightSteerMotorId = 5;
  private static final int kFrontRightEncoderId = 6;
  private static final Angle kFrontRightEncoderOffset = null;
  private static final boolean kFrontRightSteerMotorInverted = false;
  private static final boolean kFrontRightEncoderInverted = false;
  private static final Distance kFrontRightXPos = null;
  private static final Distance kFrontRightYPos = null;
  // Back Left
  private static final int kBackLeftDriveMotorId = 7;
  private static final int kBackLeftSteerMotorId = 8;
  private static final int kBackLeftEncoderId = 9;
  private static final Angle kBackLeftEncoderOffset = null;
  private static final boolean kBackLeftSteerMotorInverted = false;
  private static final boolean kBackLeftEncoderInverted = false;
  private static final Distance kBackLeftXPos = null;
  private static final Distance kBackLeftYPos = null;
  // Back Right
  private static final int kBackRightDriveMotorId = 10;
  private static final int kBackRightSteerMotorId = 11;
  private static final int kBackRightEncoderId = 12;
  private static final Angle kBackRightEncoderOffset = null;
  private static final boolean kBackRightSteerMotorInverted = false;
  private static final boolean kBackRightEncoderInverted = false;
  private static final Distance kBackRightXPos = null;
  private static final Distance kBackRightYPos = null;
  // Modules
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft = null;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight = null;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft = null;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight = null;
}
