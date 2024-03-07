package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeControlCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public IntakeControlCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
//        addRequirements(intakeSubsystem);
    }


    @Override
    public void initialize(){
        System.out.println("Intake Control Initialised");

    }

    @Override
    public void execute(){
        intakeSubsystem.intakeOn();

    }

    @Override
    public void end(boolean interrupted){

        intakeSubsystem.intakeOff();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
