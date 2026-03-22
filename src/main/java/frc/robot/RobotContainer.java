// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.AlectronaLib.SwerveDriveInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.spindexerSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveStates;
import frc.robot.subsystems.SuperStructure.ShooterStates;
import frc.robot.subsystems.SuperStructure.SpindexerStates;
import frc.robot.subsystems.shooter.shooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.shooter.shotCalculator;


import java.io.File;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import swervelib.SwerveInputStream;
// import swervelib.SwerveCont?roller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivetrain = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final shooterSubsystem m_ShooterSubsystem = new shooterSubsystem();
  private final spindexerSubsystem m_spindexer = new spindexerSubsystem();
  private final shotCalculator m_ShotCalculator = new shotCalculator(drivetrain);

  private final SuperStructure m_SuperStructure = new SuperStructure(m_ShooterSubsystem,m_spindexer,drivetrain);

  

  private final SendableChooser<Command> autoChooser;
   
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver =
      new CommandXboxController(DriveConstants.DriverPort);

  private SwerveDriveInput m_DriveInput = new SwerveDriveInput();
  private SwerveDriveInput m_RotInput = new SwerveDriveInput();

  private Translation2d modifiedDriveInput = new Translation2d(0,0);
  private Translation2d modifiedRotInput = new Translation2d(0,0);

  public void updateDriveInput(){
      modifiedDriveInput = m_DriveInput.getShapedInput(()-> driver.getLeftX(), ()-> driver.getLeftY());
      modifiedRotInput = m_RotInput.getShapedInput(()-> driver.getRightX(), ()-> driver.getRightY());

    }

  
  //Cnvert driver input into field-relative ChassisSpeeds - controlled by angular velocity
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
            () -> -modifiedDriveInput.getY()*SwerveSubsystem.getInvert(), // (-) is blue alliance
            () -> -modifiedDriveInput.getX()*SwerveSubsystem.getInvert()) // (-) is blue alliance
            .withControllerRotationAxis(() -> -modifiedRotInput.getX()) // Axis which give the desired angular velocity.
            .deadband(0.00)                 // Controller deadband
            .scaleTranslation(0.8)           // Scaled controller translation axis
            .allianceRelativeControl(true);  // Alliance relative controls.


  //Clones angular velocity input steam, converts to a fieldRelative input stream
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> Math.cos(Math.PI/3), () -> Math.sin(Math.PI/3)).headingWhile(true);

  
  //Clones angular velocity input steam, converts to a robotRelative input stream
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();

  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveAngle = drivetrain.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngle = drivetrain.driveFieldOriented(driveDirectAngle);
    Command driveRobotOrientedAngularVelocity = drivetrain.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivetrain.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    // driver.leftBumper().whileTrue(m_shooter.setState(ShooterStates.TEST));
    driver.leftBumper().whileTrue(m_ShooterSubsystem.setState(ShooterStates.TEST).alongWith(m_spindexer.setState(SpindexerStates.FEED)));
      driver.a().onTrue(drivetrain.run(()-> drivetrain.setInverted()));

      //aiming
      Command aiming = m_SuperStructure.firingCommand(ShooterStates.AIMING, SpindexerStates.OFF, DriveStates.AIMING);

      //shooting
      Command shooting = m_SuperStructure.firingCommand(ShooterStates.SHOOTING, SpindexerStates.FEED, DriveStates.AIMING);

      driver.rightBumper().whileTrue(drivetrain.SwerveControllerDrive(null,
       ()-> -modifiedDriveInput.getX(),
        ()-> -modifiedDriveInput.getY(),
         ()-> Rotation2d.fromRadians(shotCalculator.getInstance().getParameters().robotHeadingRadians()),
          null)
        );
      driver.leftTrigger().whileTrue(aiming);

      driver.rightTrigger().whileTrue(shooting);

      driver.y().onTrue(m_ShooterSubsystem.setState(ShooterStates.REZERO)); //Hood rezero

        // driver.rightBumper().whileTrue(m_ShooterSubsystem.setState(ShooterStates.IDLE));

    // drivetrain.SwerveControllerDrive(
    //         null,
    //         () -> modifiedDriveInput.getX(),
    //         () -> modifiedDriveInput.getY(),
    //         () -> {
    //             if (Math.abs(driver.getRightX()) > DriveConstants.deadband) {
    //                 return null;
    //             } else {
    //                 return drivetrain.getLastHeldRotation();
    //             }
    //         },
    //         () -> {
    //             if (Math.abs(driver.getRightX()) > DriveConstants.deadband) {
    //                 return modifiedRotInput.getX();
    //             } else {
    //                 return 0.0;
    //             }
    //         }
    //     )
    // ;

     if (RobotBase.isSimulation()) {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      drivetrain.resetOdometry(new Pose2d(3,3,new Rotation2d()));
      drivetrain.visionEnabled = false;
    } 
    else {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    driver.rightTrigger().onTrue(drivetrain.runOnce(drivetrain::zeroGyro)); //TESTING PURPOSES ONLY!!!

    // driver.rightTrigger().whileTrue(m_shooter.setState(ShooterStates.TEST)); //testing for lut values
    // driver.rightBumper().onTrue(m_shooter.setState(ShooterStates.REZERO)); //rezero hood
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake){
    drivetrain.setMotorBrake(brake);
  }
}