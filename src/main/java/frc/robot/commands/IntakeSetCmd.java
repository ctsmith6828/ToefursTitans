package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.jni.CANSWDLJNI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetCmd extends CommandBase{

    // private final IntakeSubsystem intakeSubsystem;
    // private final CANSparkMax m_IntakeForward;
    // private final CANSparkMax m_IntakeRear; 
    // private final boolean intakeForward = true;
    // private final boolean intakeEnabled = false;
    
    // public IntakeSetCmd (IntakeSubsystem intakeSubsystem) {

    //     this.intakeSubsystem = intakeSubsystem;
    //     addRequirements(intakeSubsystem);
    // }

    public void turnIntakeOn(){
//        intakeFor
    }
    @Override
    public void initialize(){
        System.out.println("Intake System Intialized");
    }

    @Override
    public void execute(){

    }
    
}
