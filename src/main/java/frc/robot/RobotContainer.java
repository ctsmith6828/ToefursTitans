// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeSetCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakSubsystem = new IntakeSubsystem();

//  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final JoystickButton button1 = new JoystickButton(driverJoystick, 1),
                 button2 = new JoystickButton(driverJoystick, 2),
                 button3 = new JoystickButton(driverJoystick, 3),
                 button4 = new JoystickButton(driverJoystick, 4),
                 button5 = new JoystickButton(driverJoystick, 5),
                 button6 = new JoystickButton(driverJoystick, 6),
                 button7 = new JoystickButton(driverJoystick, 7),
                 button8 = new JoystickButton(driverJoystick, 8);

  private final Joystick mechanismJoystick = new Joystick(OIConstants.kMechanismControllerPort);
  private final JoystickButton button11 = new JoystickButton(mechanismJoystick, 1),
                 button12 = new JoystickButton(mechanismJoystick, 2),
                 button13 = new JoystickButton(mechanismJoystick, 3),
                 button14 = new JoystickButton(mechanismJoystick, 4),
                 button15 = new JoystickButton(mechanismJoystick, 5),
                 button16 = new JoystickButton(mechanismJoystick, 6),
                 button17 = new JoystickButton(mechanismJoystick, 7),
                 button18 = new JoystickButton(mechanismJoystick, 8);

  private final PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  private final PS4Controller mechanismController = new PS4Controller(OIConstants.kDriverControllerPort);
                             
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                    swerveSubsystem, 
                    () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
                    () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
                    () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
                    () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtionIndex)));

    configureButtonBindings();

    // Configure the trigger bindings
    configureBindings();
  }

  public void configureButtonBindings(){
    if(driverController.getSquareButtonPressed()){
      
    }
         
    new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    new JoystickButton(mechanismJoystick, 3);
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
              new Translation2d(1, 0),
              new Translation2d(1, -1)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      swerveSubsystem::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController, 
      swerveSubsystem::setModuleStates, 
      swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules()));  }
}
