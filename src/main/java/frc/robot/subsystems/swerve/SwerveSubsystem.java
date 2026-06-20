// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants.limelight; 
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SwerveStates;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
// import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.util.Units;

import org.json.simple.parser.ParseException;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;


import static edu.wpi.first.units.Units.Meter;

// import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import frc.VikingsLib.EstimatePose;
import frc.VikingsLib.SwerveController;
import frc.VikingsLib.SwerveController.Speeds;
import frc.VikingsLib.LimelightHelpers;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;






public class SwerveSubsystem extends SubsystemBase {

//SwerveDrive object
  private final SwerveDrive swerveDrive;
  public static int invert = 1;

  //Apriltag field layout
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  //Disable vision when simulating
  public boolean visionEnabled = !SwerveDriveTelemetry.isSimulation;

  private final EstimatePose m_EstimatePose = new EstimatePose("limelight");

  private final SwerveController swerveController= new SwerveController(
    SwerveConstants.translationController, 
    SwerveConstants.rotationController, 
    false,
    false, 
    SwerveConstants.slewRateLimit, 
    SwerveConstants.jerkRateLimit,
    SwerveConstants.autonSlewRateLimit,
    SwerveConstants.autonJerkRateLimit
    );
  
  private final Field2d kField = new Field2d();

  private SwerveStates swerveState = SwerveStates.OPERATED;
    
  private Rotation2d lastHeldPosition = Rotation2d.fromDegrees(0);
  private boolean wasRotating = false;
  private boolean isSettlingAfterRelease = false;
  private static final double ROTATION_SETTLE_TIME = 0.3; // 300 ms
  private double rotationReleaseStartTime = -1.0;
  // private Pose2d currentPose = ;





  
  public SwerveSubsystem(File directory) {
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maxSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } 
    catch (Exception e) 
    {
      throw new RuntimeException(e);
    }

  
    swerveDrive.setChassisDiscretization(true, 0.02); //Skew reduction

    swerveDrive.setHeadingCorrection(false);// Heading correction should only be used while controlling the robot via angle.

    swerveDrive.setCosineCompensator(true);// Disables cosine compensation for simulations since it causes discrepancies not seen in real life.

     swerveDrive.setAngularVelocityCompensation(true, false, 0.1);// Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.

     swerveDrive.setModuleEncoderAutoSynchronize(true,1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.


     //Stop odometry thread if using vision -> can synchronize updates better
    //  if(visionEnabled){
    //   swerveDrive.stopOdometryThread(); 
    //  }

    setupPathPlanner();
  } 

  //Construct the swerve
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,controllerCfg, SwerveConstants.maxSpeed, new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  public void setState(SwerveStates state){
    this.swerveState = swerveState;
  }

  public SwerveStates getState(){
    return swerveState;
  }

  @Override
  public void periodic() {
    Pose2d currentPose = swerveDrive.getPose();
    kField.setRobotPose(currentPose);




    var currentState = new EstimatePose.RobotState(getFieldVelocity(), getPose());
    

    m_EstimatePose.update(currentState, (visionPose, timestamp, stdDevs) -> {
      swerveDrive.setVisionMeasurementStdDevs(stdDevs);
      swerveDrive.addVisionMeasurement(visionPose, timestamp);
    }); 


    // This method will be called once per scheduler run
    //  if (DriverStation.isAutonomous() || DriverStation.isDisabled()){
    //   mt1HeadingUpdate();
    // }
    // if (visionEnabled){
    //   updateVision();
    // }
  }

    @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }


  public void setupPathPlanner(){
    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedForward = true;
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedForward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              SwerveConstants.autoDrivePID,
              // Translation PID constants
              SwerveConstants.autoRotationPID
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preloaad PathPlanner Path finding
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

  }

  public Command getAutonomousCommand(String pathName){
    return new PathPlannerAuto(pathName);// Create a path following command using AutoBuilder. This will also trigger event markers.
  }

  public Command driveToPose(Pose2d pose){
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
      
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
    throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
            swerveDrive.getStates(),
            DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);

        });
  }

  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds){
    try
    {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    
    return Commands.none();

  }


  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(new Config(),this, swerveDrive, 12, true),
      3.0, 5.0, 3.0
      );
  }


  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(),this, swerveDrive),
        3.0, 5.0, 3.0);
  }


   //Returns a Command that centers the modules of the SwerveDrive subsystem.
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }


  //Returns a Command that drives the swerve drive to a specific distance at a given speed.
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
    .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }

  //Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  //Uses translative values and heading as ANGULAR VELOCITY
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  //Using translative values and heading as a SETPOINT
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }


  //Primary mehtod for controlling drivebase
   public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }


  //  //Drive robot given a chassis field oriented velocity
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }


  //Drive robot given a chassis field oriented velocity
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

   public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  
  //Reset odom to the given pose
  public void resetOdometry(Pose2d initialHolonomicPose){
      swerveDrive.resetOdometry(initialHolonomicPose);
    }


  //Get current pose, as reported by odometry
  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  //Set chasssis speeds with closed-loop velocity control
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  //Reset gyro angle to zero & reset odometry to same position, but facing toward 0
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  //Zero the robot to assume current position is facing forwards
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  //Set drive motors to brake/coast mode
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  //Get chassis speeds based on controller input of 2 joysticks; One for speed in a direction, other for angle of robot
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        SwerveConstants.maxSpeed);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        SwerveConstants.maxSpeed);
  }

  //Get current field-relative velocity (x, y, omega) of robot
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  //Gets current velocity (x, y, omega) of robot
 public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }


    public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  public SwerveDrive getSwerveDrive() {
      return swerveDrive;
  }

 public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> curStdDevs) {
    swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, curStdDevs);
  }

  public double getTimeSinceLastTagSeen(){
    return Timer.getFPGATimestamp() - m_EstimatePose.getLastTagTimestamp();
  };

  public void resetLatestHeading(){
    lastHeldPosition = this.getPose().getRotation();
  }

  public Rotation2d getTargetHeadingInFieldFrame(){
    final Translation2d hubPosition = Constants.fieldPoses.hubPosition();
    final Translation2d robotPosition = swerveDrive.getPose().getTranslation();
    return hubPosition.minus(robotPosition).getAngle();

  }

  public boolean isAimed(){
    return Math.abs(getTargetHeadingInFieldFrame().minus(getHeading()).getDegrees()) < SwerveConstants.aimToleranceDegrees.magnitude();
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public Command swerveLockCommand(DoubleSupplier supplier){
    return runOnce(
      () -> setState(SwerveStates.LOCKED))
      .andThen(()-> swerveDrive.lockPose()).repeatedly()
      .until(()-> supplier.getAsDouble() > Constants.DriveConstants.deadband);
  }

  @Override
  public String toString(){
    switch (swerveState) {
      case LOCKED:
        return "LOCKED";
      case AIMED:
      case LOCKED_AND_AIMED:
        return "AIM: READY";
      case AIMING:
        return "AIM: NOT READY";
      case OPERATED:
      default:
        return "OPERATED";
    }
  } 


  
  public Command swerveControllerDrive(Translation2d targetTranslation, Rotation2d targetRotation) {
    return run(() -> {
      Rotation2d currentRotation = getPose().getRotation();
      Speeds speeds = swerveController.calculate(
          swerveDrive::getPose,
          () -> new Pose2d(
          targetTranslation == null ? getPose().getTranslation() : targetTranslation,
          targetRotation == null ? getPose().getRotation() : targetRotation
          )
      );
      ChassisSpeeds finalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vx(), speeds.vy(), speeds.vr(),
          currentRotation);
      drive(finalSpeeds);
    });
  }


  

  public double getDistanceError(){
    return swerveController.getDistanceError();
  }

  public double getRotationalError(){
    return swerveController.getRotationalError();
  }

  public Rotation2d getLastHeldRotation() {
        return lastHeldPosition;
    }

}