// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;




/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriverConstants{
    public static final int kFrontLeftDriveMotorPort = 35;
    public static final int kBackLeftDriveMotorPort = 13;
    public static final int kFrontRightDriveMotorPort = 45;
    public static final int kBackRightDriveMotorPort = 41;

    public static final int kFrontLeftTurnMotorPort = 50;
    public static final int kBackLeftTurnMotorPort = 15;
    public static final int kFrontRightTurnMotorPort = 40;
    public static final int kBackRightTurnMotorPort = 20;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final Rotation2d kFrontLeftDriveAbsoluteEncoderOffsetRad = Rotation2d.fromDegrees(259.00);
    public static final Rotation2d kBackLeftDriveAbsoluteEncoderOffsetRad = Rotation2d.fromDegrees(110.00);
    public static final Rotation2d kFrontRightDriveAbsoluteEncoderOffsetRad = Rotation2d.fromDegrees(192.70);
    public static final Rotation2d kBackRightDriveAbsoluteEncoderOffsetRad = Rotation2d.fromDegrees(232.00);
  }

  public static class ModulesConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final double kWheelBase = Units.inchesToMeters(21);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    );
    public static final double kDriveMotorGearRatio = (6.75 / 1);
    public static final double kTurnMotorGearRatio = (21.42 / 1);
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurnMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurnEncoderRPM2MeterPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = .5;
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSeconds = 4.5;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSeconds = 11.5;
  }

  public static class OIConstants {
    public static final double kDeadband = 0.1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverRotAxis = 3;
    public static final int kDriverFieldOrientedButton = 11;
  }
  
}
