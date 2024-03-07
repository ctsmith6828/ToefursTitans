package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;

public class ClimbSubsystem {

    private final CANSparkMax m_ClimbLeft = new CANSparkMax(ClimbConstants.m_CLIMB_LEFT_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax m_ClimbRight = new CANSparkMax(ClimbConstants.m_CLIMB_RIGHT_CAN_ID, MotorType.kBrushless);

    public ClimbSubsystem(){
        m_ClimbLeft.setInverted(false);
        m_ClimbRight.setInverted(true);

    }

    public void climbOn(){
        m_ClimbLeft.setInverted(false);
        m_ClimbRight.setInverted(true);
        m_ClimbLeft.set(ClimbConstants.CLIMB_POWER);
        m_ClimbRight.set(ClimbConstants.CLIMB_POWER);
    }

    public void climbUp(){
        m_ClimbLeft.setInverted(true);
        m_ClimbRight.setInverted(false);
        m_ClimbLeft.set(ClimbConstants.CLIMB_UP_POWER);
        m_ClimbRight.set(ClimbConstants.CLIMB_UP_POWER);
    }

    public void climbOff() {
        m_ClimbLeft.set(0);
        m_ClimbRight.set(0);
    }
    
}
