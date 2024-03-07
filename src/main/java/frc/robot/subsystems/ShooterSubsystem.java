package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem {
 
    private final CANSparkMax m_ShooterLeft = new CANSparkMax(ShooterConstants.m_SHOOTER_LEFT_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax m_ShooterRight = new CANSparkMax(ShooterConstants.m_SHOOTER_RIGHT_CAN_ID, MotorType.kBrushless);

    public ShooterSubsystem(){
        m_ShooterLeft.setInverted(true);
        m_ShooterRight.setInverted(true);

    }

    public void shooterOn(double power){
        m_ShooterLeft.set(power);
        m_ShooterRight.set(power);
    }

    public void shooterOff(){
        m_ShooterLeft.set(0);
        m_ShooterRight.set(0);
    }

}
