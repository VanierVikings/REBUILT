
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;;

import frc.robot.subsystems.shooter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = 0.0; // kg

  public static class SwerveConstants{
    public static final double maxSpeed = Units.feetToMeters(9); 
    public static final double kP = 4;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final PIDController translationController = new PIDController(4, 0, 0.15);  //OP ROBOTICS PID : 4, 0, 0.15
    public static final PIDController rotationController = new PIDController(4,0,0.15);
    public static final double jerkRateLimit = 45;
    public static final double slewRateLimit = 20;
    public static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static final PIDConstants autoDrivePID = new PIDConstants(0,0,0);
    public static final PIDConstants autoRotationPID = new PIDConstants(0,0,0);
  }
  
  public static class ShooterConstants {
    public static final int shooterMotorTopMasterID = 0;
    public static final int shooterMotorTopSlaveID = 0;
    public static final int shooterMotorBottomID = 0;
    public static final double kShooterP = 0.0;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterFF = 0.0;

    public static final double kHoodP = 0.0;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.0;
  

    public static final int SHOOTER_CURRENT_LIMIT = 40; //amps;
    public static final int HOOD_CURRENT_LIMIT = 40; //made this up

    public static final int maxDistance = 10; // I put random values for these, used to make sure hood does not kill itself 
    public static final int minDistance = 1;

    public static final int degreesPerRotation = 365/30*360; //no clue how to do this, probabaly wrong

  }

    public static class SpindexerConstants {
      public static final int SPINDEXER_MOTOR_ID = 0;
      public static final int SPINDEXER_CURRENT_LIMIT = 40;

      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

  }

  public static class OperatorConstants {
    public static final int DriverPort = 0;
    public static final double deadband = 0.01;
  }
}
