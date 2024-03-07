// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double RIGHT_Y_DEADBAND = .1;
  }

  public static class ShooterConstants{
    public static final int m_SHOOTER_LEFT_CAN_ID = 20;
    public static final int m_SHOOTER_RIGHT_CAN_ID = 21;
    public static final int m_SHOOTER_ANGLE_CAN_ID = 26;
    public static final double SHOOTER_SPEAKER_POWER = 0.85;
    public static final double SHOOTER_AMP_POWER = 0.65;
    public static final double SHOOTER_TRAP_POWER = 0.40;
    public static final double ROTATE_POWER = 0.5;

    public static final int SHOOTER_ANGLE_AMP = 1;
    public static final int SHOOTER_ANGLE_SPEAKER = 100;
    public static final int SHOOTER_ANGLE_TRAP = 1;
    public static final int SHOOTER_ANGLE_UP = 1;
    public static final int SHOOTER_ANGLE_DOWN = 1;
  }

  public static class IntakeConstants{
    public static final int m_INTAKE_FRONT_CAN_ID = 22;
    public static final int m_INTAKE_BACK_CAN_ID = 23;
    public static final double INTAKE_POWER = 0.5;
    public static final double INTAKE_REVERSE_POWER = -1;
    public static final int m_SHOOTER_PIVOT_CAN_ID = 26;
  }

  public static class ClimbConstants{
    public static final int m_CLIMB_LEFT_CAN_ID = 24;
    public static final int m_CLIMB_RIGHT_CAN_ID = 25;
    public static final double CLIMB_POWER = .6;
    public static final double CLIMB_UP_POWER = .3;
  }

}
