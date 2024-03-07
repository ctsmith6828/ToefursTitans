package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class ShooterAngleSubsystem {
 
    // initialize SPARK MAX with CAN ID
    private final CANSparkMax m_ShooterAngle;
    private final RelativeEncoder shooterAngleEncoder;
    private double targetPosition=0;
    private SparkPIDController m_PidController;
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    
    public ShooterAngleSubsystem(){
        m_ShooterAngle= new CANSparkMax(ShooterConstants.m_SHOOTER_ANGLE_CAN_ID, MotorType.kBrushed);
        m_ShooterAngle.setInverted(false);
        shooterAngleEncoder = m_ShooterAngle.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7);
        m_ShooterAngle.restoreFactoryDefaults();

    m_ShooterAngle.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_PidController = m_ShooterAngle.getPIDController();
  
    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */
    m_PidController.setFeedbackDevice(shooterAngleEncoder);

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Shooter Intake Angle - P Gain", kP);
    SmartDashboard.putNumber("Shooter Intake Angle - I Gain", kI);
    SmartDashboard.putNumber("Shooter Intake Angle - D Gain", kD);
    SmartDashboard.putNumber("Shooter Intake Angle - I Zone", kIz);
    SmartDashboard.putNumber("Shooter Intake Angle - Feed Forward", kFF);
    SmartDashboard.putNumber("Shooter Intake Angle - Max Output", kMaxOutput);
    SmartDashboard.putNumber("Shooter Intake Angle - Min Output", kMinOutput);
    SmartDashboard.putNumber("Shooter Intake Angle - Set Rotations", 0);

    }

    public double getEncoderValue(){
        return shooterAngleEncoder.getPosition();
    }

    public void setTargetAngle(double targetPosition){
        this.targetPosition = targetPosition;

    }

    public void runToPosition(double targetposition){
        m_PidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    
        SmartDashboard.putNumber("Shooter Intake Angle - SetPoint", targetPosition);
        SmartDashboard.putNumber("Shooter Intake Angle - ProcessVariable", shooterAngleEncoder.getPosition());
        }

}
