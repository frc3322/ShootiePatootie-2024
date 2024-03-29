// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CANIds {
    // SPARK MAX and FLEX CAN IDs
    public static final int kFrontLeftDrivingCanId = 47;
    public static final int kFrontLeftTurningCanId = 13;

    public static final int kFrontRightDrivingCanId = 48;
    public static final int kRearRightTurningCanId = 12;

    public static final int kRearLeftDrivingCanId = 49;
    public static final int kRearLeftTurningCanId = 23;

    public static final int kRearRightDrivingCanId = 51;
    public static final int kFrontRightTurningCanId = 36;


    // Intake CAN IDs

    public static final int kIntakeCanID = 10;
    public static final int kIntakeTurnPitchCanID = 29;

    // Shooter IDs
    public static final int kShooterAngleMotorCanId = 8;
    public static final int kShooterLeftMotorCanId = 59;
    public static final int kShooterRightMotorCanId = 62;

    //
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 9; // radians per second //old is 9
    public static final double kMagnitudeSlewRate = 5; // percent per second (1 = 100%) //old is 2.6
    public static final double kRotationalSlewRate = 3.0; // percent per second (1 = 100%) //0ld is 2.0

    // Chassis configuration
    // Distance between centers of right and left wheels on robots
    public static final double kTrackWidth = Units.inchesToMeters(24.375);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.375);
    // Distance from center of robot to furthest wheel
    public static final double kWheelRadius = Units.inchesToMeters(34.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Works now, reverses gyro everywhere in drivetrain
    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class FloopIntakeConstants {
    public static final int kIntakePistonOut = 1;
    public static final int kIntakePistonIn = 2;
    public static final double intakeP = 0;
    public static final double intakeI = 0;
    public static final double intakeD = 0;
    public static final double velocityConstraint = 0;
    public static final double accelerationConstraint = 0;

    

    public static final class TransferConstants {
    }

    public static final class ShooterConstants {

      public static final double shooterTopP = 0;
      public static final double shooterTopI = 0;
      public static final double shooterTopD = 0;
      public static final double SHOOT = -0.15;

    }

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.09;
    public static int kSecondaryControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAccelerationMetersPerSecondSquaredSlow = 0.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double kPHoloTranslationController = 5;
    public static final double kPHoloRotationController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
                                                                                                                   // this
                                                                                                                   // should
                                                                                                                   // likely
                                                                                                                   // live
                                                                                                                   // in
                                                                                                                   // your
                                                                                                                   // Constants
                                                                                                                   // class
        new PIDConstants(kPHoloTranslationController, 0.0, 0.0), // Translation PID constants
        new PIDConstants(kPHoloRotationController, 0.0, 0.0), // Rotation PID constants
        4.5, // Max module speed, in m/s
        DriveConstants.kWheelRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    public static final PathConstraints constraints = new PathConstraints(kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared,
        Units.radiansToDegrees(kMaxAngularSpeedRadiansPerSecond),
        Units.radiansToDegrees(kMaxAngularSpeedRadiansPerSecondSquared));
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;

    public static final int neo550CurrentLimitAmps = 20;
  }

  public static final class FieldConstants {
    // Starting note transforms - Top is amp. Notes are 5ft 6in apart, with one
    // centered on the line
    public static final Translation2d centerTopPose = new Translation2d(0, Units.feetToMeters(11));
    public static final Translation2d centerMidTopPose = new Translation2d(0, Units.feetToMeters(5.5));
    public static final Translation2d centerMidPose = new Translation2d(0, Units.feetToMeters(11));
    public static final Translation2d centerMidBottomPose = new Translation2d(0, Units.feetToMeters(-5.5));
    public static final Translation2d centerBottomPose = new Translation2d(0, Units.feetToMeters(-11));

    // Amp poses. Both halves of the field together are 651.25 in long, and amp is 4
    // ft 1.5 in from the wall.
    // The top wall that the amp is in is 161.625 from the center of the field.
    // Needs to have an offset subtracted from it later
    public static final Pose2d redAmpPose = new Pose2d(
        new Translation2d(Units.inchesToMeters(325.625 - 49.5), Units.inchesToMeters(161.625)),
        new Rotation2d(-90));

    public static final Pose2d blueAmpPose = new Pose2d(
        new Translation2d(-Units.inchesToMeters(325.625 - 49.5), Units.inchesToMeters(161.625)),
        new Rotation2d(90));
  }
}
