// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;


public class SwerveSubsystem extends SubsystemBase {

//SwerveDrive object
  private final SwerveDrive swerveDrive;

  //Apriltag field layout
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  //Disable vision when simulating
  public boolean visionEnabled = !SwerveDriveTelemetry.isSimulation;

  public SwerveController controller;

  
  public SwerveSubsystem(File directory) {
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    
    try{
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } 
    catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);// Heading correction should only be used while controlling the robot via angle.

    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);// Disables cosine compensation for simulations since it causes discrepancies not seen in real life.

     swerveDrive.setAngularVelocityCompensation(false, false, 0);// Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.

     swerveDrive.setModuleEncoderAutoSynchronize(false,1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.

     if(visionEnabled){
      swerveDrive.stopOdometryThread(); //self explanatory
     }

     setupPathPlanner();

  }

  //Construct the swerve
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveDriveConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveConstants.MAX_SPEED, new )
  }



    


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
      return swerveDrive;
  }
}
