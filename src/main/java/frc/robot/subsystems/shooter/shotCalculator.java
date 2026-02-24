package frc.robot.subsystems.shooter;

import java.io.File;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;


public class shotCalculator  extends SubsystemBase{
    private static shotCalculator instance;
    private SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    
        private double lastHoodAngle = Double.NaN;
        private ShootParameters latestParameters = null;
    
        public shotCalculator(SwerveSubsystem drive){
            this.drive = drive;
    }

    public static shotCalculator getInstance(SwerveSubsystem drive) {
        if (instance == null) instance = new shotCalculator(drive);
        return instance;
    }

    public static shotCalculator getInstance(){
        return instance;
    }


    /**
     * Data record containing all calculated parameters for a shot.
     */
    public record ShootParameters(
        double initialDistance,
        double hoodAngle,
        double flywheelSpeed,
        double lookaheadDistance,
        double robotHeadingRadians, // Field-relative angle in radians
        boolean isPassing    
        ) {}


    private static final InterpolatingTreeMap<Double, Rotation2d> passingShotHoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap passingShotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    static{

        //test values
        passingShotHoodAngleMap.put(0.0,Rotation2d.fromDegrees(0)); //distance, angle

        passingShotFlywheelSpeedMap.put(0.0,0.0); //distance, RPS
    }

    // Interpolation maps for shot lookups
    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static{
        shotHoodAngleMap.put(0.0, Rotation2d.fromDegrees(0.0)); // distance, angle

        shotFlywheelSpeedMap.put(0.0, 0.0); //distance RPS
    }


    @Override
    public void periodic(){
        clearShootingParameters();
        SmartDashboard.putNumber("Distance to hub", getInstance().getParameters().initialDistance);
        SmartDashboard.putNumber("Desired Robot Heading", Units.radiansToDegrees(getInstance().getParameters().robotHeadingRadians));
        SmartDashboard.putNumber("Desired Hood Angle", getInstance().getParameters().hoodAngle);
    }

    public ShootParameters getParameters(){
        if(latestParameters ==null){
            return latestParameters;
        }

        //1. Determine target based on alliance
        var alliance = DriverStation.getAlliance();

        //2. Get current robot/shooter position
        Pose2d currentPose = drive.getPose();

        InterpolatingTreeMap<Double, Rotation2d> currentShotHoodAngleMap;
        InterpolatingDoubleTreeMap currentShotFlywheelSpeedMap;
        InterpolatingDoubleTreeMap currentTimeOfFlightMap;

        boolean isRed = alliance.orElse(Alliance.Blue) == Alliance.Red;
        double robotX = currentPose.getX();

        boolean isPastTrench = isRed ? (robotX < 11) : (robotX > 5.5);
        boolean isLeftRight = currentPose.getY() > 4;

        Pose2d targetPose;

         if (isPastTrench) {
            targetPose = isLeftRight ? new Pose2d(1, 7, new Rotation2d()) : new Pose2d(1, 1, new Rotation2d());
            if (isRed) targetPose = FlippingUtil.flipFieldPose(targetPose);
            currentShotHoodAngleMap = passingShotHoodAngleMap;
            currentShotFlywheelSpeedMap = passingShotFlywheelSpeedMap;
            currentTimeOfFlightMap = passingTimeOfFlightMap;
        } else {
            targetPose = isRed ? Constants.fieldPoses.redAllianceHub : Constants.fieldPoses.blueAllianceHub;
             currentShotHoodAngleMap = shotHoodAngleMap;
            currentShotFlywheelSpeedMap = shotFlywheelSpeedMap;
            currentTimeOfFlightMap = timeOfFlightMap;
        }


        Pose2d shooterPosition = currentPose.transformBy(
            new Transform2d(
                Constants.robotToShooter.getTranslation().toTranslation2d(),
                Constants.robotToShooter.getRotation().toRotation2d()
            )
        );
        double shooterToTargetDistance = targetPose.getTranslation().getDistance(shooterPosition.getTranslation());
        SmartDashboard.putNumber("shooterToTargetDistance", shooterToTargetDistance);

        // 3. Calculate Field-Relative Velocity of the Shooter
        ChassisSpeeds fieldSpeeds = drive.getFieldVelocity();
        double robotRotation = currentPose.getRotation().getRadians();
        Translation2d shooterOffset = Constants.robotToShooter.getTranslation().toTranslation2d();

        // Account for tangential velocity due to robot rotation
        double shooterVelocityX = fieldSpeeds.vxMetersPerSecond
            + fieldSpeeds.omegaRadiansPerSecond
            * (shooterOffset.getY() * Math.cos(robotRotation) - shooterOffset.getX() * Math.sin(robotRotation));
        
        double shooterVelocityY = fieldSpeeds.vyMetersPerSecond
            + fieldSpeeds.omegaRadiansPerSecond
            * (shooterOffset.getX() * Math.cos(robotRotation) - shooterOffset.getY() * Math.sin(robotRotation));

        // --- Inside getParameters() after calculating shooterVelocityX/Y ---

        //Initial TOF guess
        double initialDistance = targetPose.getTranslation().getDistance(shooterPosition.getTranslation());
        double predictedTOF = currentTimeOfFlightMap.get(initialDistance);

        // Iteratively find the VIRTUAL TARGET
        // We subtract the velocity from the target's position to compensate for ball momentum
        Translation2d virtualTarget = targetPose.getTranslation(); 

        for (int i = 0; i < 3; i++) { //99.9% convergence
            virtualTarget = targetPose.getTranslation().minus(
                new Translation2d(shooterVelocityX * predictedTOF, shooterVelocityY * predictedTOF)
            );
            double virtualDistance = virtualTarget.getDistance(shooterPosition.getTranslation());
            predictedTOF = currentTimeOfFlightMap.get(virtualDistance);
        }

        // final calculations based on the Virtual Target
        double lookaheadDistance = virtualTarget.getDistance(shooterPosition.getTranslation());

        // robot points at the Virtual Target (this handles the leading/sideways drift)
        Rotation2d targetAngle = virtualTarget.minus(shooterPosition.getTranslation()).getAngle();
        double robotHeadingRadians = targetAngle.getRadians();

        // Interpolate based on the distance to that virtual target
        double hoodAngle = currentShotHoodAngleMap.get(lookaheadDistance).getDegrees();
        double flywheelSpeed = currentShotFlywheelSpeedMap.get(lookaheadDistance);

        //Calculate Target Hood Velocity (Rate of change)
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
        lastHoodAngle = hoodAngle;


        latestParameters = new ShootParameters(
            initialDistance,
            hoodAngle,
            flywheelSpeed,
            lookaheadDistance,
            robotHeadingRadians,
            isPastTrench
        );

        return latestParameters;
    }

    /**
     * Resets the cached parameters. Call this at the start of every robot loop.
     */
    public void clearShootingParameters() {
        latestParameters = null;
    }

}

