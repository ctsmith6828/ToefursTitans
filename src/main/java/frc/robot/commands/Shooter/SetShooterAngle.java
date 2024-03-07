package frc.robot.commands.Shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.naming.InitialContext;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class SetShooterAngle extends Command {
    private ShooterAngleSubsystem shooterAngleSubsystem;
    private double targetShooterAngle;

    public SetShooterAngle(ShooterAngleSubsystem shooterAngleSubsystem, double setShooterAngle) {
        this.shooterAngleSubsystem = shooterAngleSubsystem;

        this.targetShooterAngle = setShooterAngle;
        addRequirements();

    }

    @Override
    public void initialize() {
        System.out.println("SetShooterAngle Initialized");
    }

    @Override
    public void execute(){
        shooterAngleSubsystem.runToPosition(targetShooterAngle);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
