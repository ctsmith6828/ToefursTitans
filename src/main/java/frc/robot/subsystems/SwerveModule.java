// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;
  private final RelativeEncoder driveEncoder; 
  private final RelativeEncoder turningEncoder;
  private final CANcoder absoluteEncoder;

  private final PIDController turningPIDController;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  /** Creates a new SwerveModuel. */
  public SwerveModule(int driveMotorCanId, int turningMotorCanId, boolean driveMotorReversed, boolean turningMotorReversed, 
          int absoluteEncoderCanId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderCanId);

    driveMotor = new CANSparkMax(driveMotorCanId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorCanId, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    turningPIDController = new PIDController(ModuleConstants.kPTurning, absoluteEncoderCanId, absoluteEncoderOffset);
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
    }

    public double getDrivePosition(){
      return driveEncoder.getPosition();
    }


    public double getTurningPosition(){
      return turningEncoder.getPosition();
    }
    
    public double getDriveVelocity(){
      return driveEncoder.getVelocity();
    }
    
    public double getTurningVelocity(){
      return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
      double angle = absoluteEncoder.getPosition().getValueAsDouble();
      angle *= 2.0 * Math.PI;
      angle -= absoluteEncoderOffsetRad;
      return angle * (absoluteEncoderReversed ? -1: 1);
    }

    public void resetEncoders(){
      driveEncoder.setPosition(0);
      turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
      if(Math.abs(state.speedMetersPerSecond) < 0.001){
        stop();
        return;
      }
      state = SwerveModuleState.optimize(state, getState().angle);    // keeps the module from turning more than 90 degrees
      driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
      turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop(){
      driveMotor.set(0);
      turningMotor.set(0);
    }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
