// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Intake.IntakeControlCommand;
import frc.robot.commands.Shooter.SetShooterAngle;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.fasterxml.jackson.databind.node.ShortNode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  public static boolean shooterRunning = false;
  public static boolean intaking = false;
  public Thread shooterThread = new Thread();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final ShooterAngleSubsystem shooterAngleSubsystem = new ShooterAngleSubsystem();
//  private final SetShooterAngle setShooterAngle;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final Joystick joystick = new Joystick(3);
  final CommandXboxController driverTwo = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> 1-2 * Math.abs(MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_Y_DEADBAND)));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

        
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureButtonBindings(){

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.b().whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    driverXbox.a().toggleOnTrue(Commands.runOnce(() -> {
      intakeSubsystem.intakeOn();
    }));
    driverXbox.a().toggleOnFalse(Commands.runOnce(intakeSubsystem::intakeOff));
    driverXbox.b().toggleOnTrue(Commands.runOnce(intakeSubsystem::intakeReverse));
    driverXbox.b().toggleOnFalse(Commands.runOnce(intakeSubsystem::intakeOff));
    driverXbox.leftBumper().toggleOnTrue(Commands.runOnce(intakeSubsystem::intakeReverse));
    driverXbox.leftBumper().toggleOnFalse(Commands.runOnce(intakeSubsystem::intakeOff));
    driverTwo.b().toggleOnTrue(Commands.runOnce(() -> {
      shooterSubsystem.shooterOff();
    }));
    driverTwo.a().toggleOnTrue(Commands.runOnce(() -> {
      shooterSubsystem.shooterOn(ShooterConstants.SHOOTER_SPEAKER_POWER);
      // setShooterAngle.crudeRunToPosition(100);
    }));
    driverTwo.y().toggleOnTrue(Commands.runOnce(() -> {
      shooterSubsystem.shooterOn(ShooterConstants.SHOOTER_AMP_POWER);
      // setShooterAngle.crudeRunToPosition(100);
    }));
    driverTwo.rightTrigger(0).onTrue(Commands.runOnce(() -> {
      shooterSubsystem.shooterOn(driverTwo.getRightTriggerAxis());
    }));
    driverTwo.rightTrigger().onFalse(Commands.runOnce(() -> {
      // shooterSubsystem.shooterOff();
      shooterSubsystem.shooterOn(driverTwo.getRightTriggerAxis());

    }));
    driverTwo.povUp().toggleOnTrue(Commands.runOnce(() -> {
      climbSubsystem.climbUp();
    }));
    driverTwo.povUp().toggleOnFalse(Commands.runOnce(() -> {
      climbSubsystem.climbOff();
    }));
    driverTwo.povDown().toggleOnTrue(Commands.runOnce(() -> {
      climbSubsystem.climbOn();
    }));
    driverTwo.povDown().toggleOnFalse(Commands.runOnce(() -> {
      climbSubsystem.climbOff();
    }));
    driverTwo.a().toggleOnTrue(Commands.runOnce(() -> {
      new SetShooterAngle(shooterAngleSubsystem, ShooterConstants.SHOOTER_ANGLE_SPEAKER);
    }));
    driverTwo.povLeft().toggleOnTrue(Commands.runOnce(() -> {
      // new SetShooterAngle(shooterAngleSubsystem, ShooterConstants.SHOOTER_ANGLE_SPEAKER);
    }));
    driverTwo.povLeft().toggleOnFalse(Commands.runOnce(() -> {
      // new SetShooterAngle(shooterAngleSubsystem, ShooterConstants.SHOOTER_ANGLE_AMP);
    }));
    driverTwo.povRight().toggleOnTrue(Commands.runOnce(() -> {
      // new SetShooterAngle(shooterAngleSubsystem, ShooterConstants.SHOOTER_ANGLE_SPEAKER);
    }));
    driverTwo.povRight().toggleOnFalse(Commands.runOnce(() -> {
      // new SetShooterAngle(shooterAngleSubsystem, ShooterConstants.SHOOTER_ANGLE_SPEAKER);
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
