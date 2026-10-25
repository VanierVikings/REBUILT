// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.AlectronaLib.SwerveDriveInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.spindexerSubsystem;
import frc.robot.subsystems.shooter.shooterSubsystem;
import frc.robot.subsystems.shooter.shotCalculator;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;
// import swervelib.SwerveController;

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
  private final intakeSubsystem m_intake = new intakeSubsystem();

  private final SuperStructure m_SuperStructure = new SuperStructure(m_ShooterSubsystem,m_spindexer,m_intake,drivetrain);
  
  private final SendableChooser<Command> autoChooser;
   
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(DriveConstants.DriverPort);
  private final CommandXboxController operator = new CommandXboxController(1);

  private SwerveDriveInput m_DriveInput = new SwerveDriveInput();
  private SwerveDriveInput m_RotInput = new SwerveDriveInput();

  private Translation2d modifiedDriveInput = new Translation2d(0,0);
  private Translation2d modifiedRotInput = new Translation2d(0,0);

  public void updateDriveInput(){
      modifiedDriveInput = m_DriveInput.getShapedInput(()-> -driver.getLeftX(), ()-> -driver.getLeftY());
      modifiedRotInput = m_RotInput.getShapedInput(()-> driver.getRightX(), ()-> driver.getRightY());
  }

  // Convert driver input into field-relative ChassisSpeeds - controlled by angular velocity
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
            () -> modifiedDriveInput.getY(), // (-) is blue alliance
            () -> modifiedDriveInput.getX()) // (-) is blue alliance
            .withControllerRotationAxis(() -> modifiedRotInput.getX()) // Axis which give the desired angular velocity.
            .deadband(0.00)                 // Controller deadband
            .scaleTranslation(0.8)           // Scaled controller translation axis
            .allianceRelativeControl(true);  // Alliance relative controls.
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // NamedCommands.registerCommand("aimAndSpinup", m_SuperStructure.aimingCommand(ShooterStates.AIMING, SpindexerStates.OFF, DriveStates.AIMING));
    // NamedCommands.registerCommand("shootingCommand", m_SuperStructure.aimingCommand(ShooterStates.SHOOTING, SpindexerStates.FEED, DriveStates.AIMING));

    // Configure the trigger bindings
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choices", autoChooser);
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
    Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);

    // Command aiming = m_SuperStructure.aimCommand(
    //     () -> modifiedDriveInput.getX(), () -> modifiedDriveInput.getY());
    // Command shooting = m_SuperStructure.shootCommand(
    //     () -> modifiedDriveInput.getX(), () -> modifiedDriveInput.getY());
    // Trigger rightTrigger = driver.rightTrigger();
    // driver.leftTrigger().and(rightTrigger.negate()).whileTrue(aiming);
    // rightTrigger.whileTrue(shooting);
    
    driver.leftBumper().onTrue(m_SuperStructure.toggleIntakeCommand());
    driver.leftTrigger().and(driver.rightTrigger().negate()).whileTrue(m_SuperStructure.aimCommand(() -> modifiedDriveInput.getY(), () -> modifiedDriveInput.getX()));
    driver.rightTrigger().whileTrue(m_SuperStructure.shootCommand(() -> modifiedDriveInput.getY(), () -> modifiedDriveInput.getX()));

    driver.y().onTrue(drivetrain.runOnce(drivetrain::zeroGyro));

     if (RobotBase.isSimulation()) {
      drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      drivetrain.resetOdometry(new Pose2d(3,3,new Rotation2d()));
      drivetrain.visionEnabled = false;
    } else {
        drivetrain.setDefaultCommand(driveFieldOrientedAnglularVelocity
        //   drivetrain.SwerveControllerDrive(
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
        //         },
        //       true
        //     )
        );

    }
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

  SuperStructure getSuperStructureForTest() { return m_SuperStructure; }
  shooterSubsystem getShooterForTest() { return m_ShooterSubsystem; }
  spindexerSubsystem getSpindexerForTest() { return m_spindexer; }
  intakeSubsystem getIntakeForTest() { return m_intake; }
  SwerveSubsystem getDrivetrainForTest() { return drivetrain; }
}