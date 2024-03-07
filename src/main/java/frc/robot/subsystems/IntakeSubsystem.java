package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem {

    private final CANSparkMax intakeFront = new CANSparkMax(IntakeConstants.m_INTAKE_FRONT_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax intakeBack = new CANSparkMax(IntakeConstants.m_INTAKE_BACK_CAN_ID, MotorType.kBrushless);

    public IntakeSubsystem(){
        intakeFront.setInverted(false);
        intakeBack.setInverted(true);
    }

    public void intakeOn(){
        intakeFront.set(IntakeConstants.INTAKE_POWER);
        intakeBack.set(IntakeConstants.INTAKE_POWER);
    }

    public void intakeReverse(){
        intakeFront.set(IntakeConstants.INTAKE_REVERSE_POWER);
        intakeBack.set(IntakeConstants.INTAKE_REVERSE_POWER);
    }
    public void intakeOff(){
        intakeFront.set(0);
        intakeBack.set(0);
    }

    
}
