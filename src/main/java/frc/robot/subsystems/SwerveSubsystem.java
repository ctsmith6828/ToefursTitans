// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftAbsoluteEncoderCanId,
        DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftAbsoluteEncoderReversed);

  private final SwerveModule rearLeft = new SwerveModule(
          DriveConstants.kRearLeftDriveCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed,
          DriveConstants.kRearLeftAbsoluteEncoderCanId,
          DriveConstants.kRearLeftAbsoluteEncoderOffsetRad,
          DriveConstants.kRearLeftAbsoluteEncoderReversed);
  
  private final SwerveModule frontRight = new SwerveModule(
          DriveConstants.kFrontRightDriveCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          DriveConstants.kFrontRightAbsoluteEncoderCanId,
          DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
          DriveConstants.kFrontRightAbsoluteEncoderReversed);
    
  private final SwerveModule rearRight = new SwerveModule(
          DriveConstants.kRearRightDriveCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed,
          DriveConstants.kRearRightAbsoluteEncoderCanId,
          DriveConstants.kRearRightAbsoluteEncoderOffsetRad,
          DriveConstants.kRearRightAbsoluteEncoderReversed);

  private final ChassisSpeeds chassisSpeed = new ChassisSpeeds();

  // private AHRS gyro = new AHRS(SPI.Port.KXP);
  private BuiltInAccelerometer gyro = new BuiltInAccelerometer();
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), null);

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){

      }
    }).start();


  }

  public void zeroHeading() {

    //ToDo: Make sure that the gyro heading gets set to 0
    //gyro.reset();
  }

  public double getHeading(){
    return gyro.getZ();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }


  public void resetOdometry(Pose2d pose) {

    // ToDo: Figure out SwerveModulePositions
    // odometer.resetPosition(pose, getRotation2d());
}

  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    rearRight.stop();
    rearLeft.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredState){
    //ToDo: Fix the normalizeWheelSpeeds method
//    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, chassisSpeeds, getDriveVelocity, ModuleConstants.kPhysicalMaxSpeedMetersPerSecond, getHeading(), getHeading(), getHeading());
//    SwerveDriveKinematics.normalizeWheelSpeeds(desiredState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredState[0]);
    frontRight.setDesiredState(desiredState[1]);
    rearLeft.setDesiredState(desiredState[2]);
    rearRight.setDesiredState(desiredState[3]);

  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ToDo: Fix the odometry update method
    // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), rearLeft.getState(),
    //             rearRight.getState());
    //     SmartDashboard.putNumber("Robot Heading", getHeading());
    //     SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("Heading = ", getHeading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
