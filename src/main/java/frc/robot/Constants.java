// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public static class ModuleConstants{

    public static final double kWheelDimeterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/5.9462; 
    public static final double kTurningMotorGearRatio = 1/18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDimeterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRot2Rad = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kPTurning = 0.5;
  }


  public static class DriveConstants{
    public static final double kTrackWidth = Units.inchesToMeters(25);  // Distance betwen right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25);   // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2));

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    

    
    // Front Left Module constants
    public static final int kFrontLeftDriveCanId = 11;
    public static final int kFrontLeftTurningCanId = 10;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final int kFrontLeftAbsoluteEncoderCanId = 8;
    public static final double kFrontLeftAbsoluteEncoderOffsetRad = 0;
    public static final boolean kFrontLeftAbsoluteEncoderReversed = false;

    // Rear Left Module constants
    public static final int kRearLeftDriveCanId = 13;
    public static final int kRearLeftTurningCanId = 12;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final int kRearLeftAbsoluteEncoderCanId = 7;
    public static final double kRearLeftAbsoluteEncoderOffsetRad = 0;
    public static final boolean kRearLeftAbsoluteEncoderReversed = false;

    // Front Right Module constants
    public static final int kFrontRightDriveCanId = 15;
    public static final int kFrontRightTurningCanId = 14;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final int kFrontRightAbsoluteEncoderCanId = 9;
    public static final double kFrontRightAbsoluteEncoderOffsetRad = 0;
    public static final boolean kFrontRightAbsoluteEncoderReversed = false;
    
    // Rear Right Module constants
    public static final int kRearRightDriveCanId = 17;
    public static final int kRearRightTurningCanId = 16;
    public static final boolean kRearRightDriveEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;
    public static final int kRearRightAbsoluteEncoderCanId = 6;
    public static final double kRearRightAbsoluteEncoderOffsetRad = 0;
    public static final boolean kRearRightAbsoluteEncoderReversed = false;  
    
  }

  public static class OIConstants{
    public static final int kMechanismControllerPort = 0;
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtionIndex = 1;

    public static final double kDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
}
  public static final class MechanismConstants{
    public static final int kShooterMotorRightCanId =  21;
    public static final int kShooterMotorLeftCanId =  20;
    public static final int kIntakeMotorFrontCanId =  22;
    public static final int kIntakeMotorRearCanId =  23;
    public static final int kClimbMotorLeftCanId =  24;
    public static final int kClimbMotorRightCanId =  25;
    public static final int kShooterPivotCanId = 26;

  }
}
