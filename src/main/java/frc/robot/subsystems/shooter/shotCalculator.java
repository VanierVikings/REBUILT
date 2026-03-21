package frc.robot.subsystems.shooter;

import java.util.concurrent.TimeoutException;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Combines high-fidelity pose extrapolation with custom shot maps.
 */
    public class shotCalculator extends SubsystemBase {
        private static shotCalculator instance;
        private final SwerveSubsystem drive;

        private ShootParameters latestParameters = null;
        private double lastHoodAngle = Double.NaN;
        private final LinearFilter hoodVelocityFilter = LinearFilter.movingAverage(5);

        private static final double PHASE_DELAY_SEC = 0.03; 
        private static final double LOOP_PERIOD = 0.02;

        // --- Shot Maps (Preserved from your original code) ---
        private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
        private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
        private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

        private static final InterpolatingTreeMap<Double, Rotation2d> passingShotHoodAngleMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
        private static final InterpolatingDoubleTreeMap passingShotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
        private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap = new InterpolatingDoubleTreeMap();
        private static Pose2d targetPose;
        private static InterpolatingTreeMap<Double, Rotation2d> currentShotHoodAngleMap;
        private static InterpolatingDoubleTreeMap currentShotFlywheelSpeedMap;
        private static InterpolatingDoubleTreeMap currentTimeOfFlightMap;
                Translation2d shooterOffset = Constants.robotToShooter.getTranslation().toTranslation2d();


    static {
        // // Shooting Maps
            shotHoodAngleMap.put(3.67, Rotation2d.fromDegrees(0));
            shotFlywheelSpeedMap.put(3.67, 50.0);
            // timeOfFlightMap.put(3.67, null);

            shotHoodAngleMap.put(2.84, Rotation2d.fromDegrees(0));
            shotFlywheelSpeedMap.put(2.84, 45.0);

            shotHoodAngleMap.put(2.33, Rotation2d.fromDegrees(0));
            shotFlywheelSpeedMap.put(2.33, 35.0);
            timeOfFlightMap.put(2.33,1.29);

            shotHoodAngleMap.put(4.189, Rotation2d.fromDegrees(4));
            shotFlywheelSpeedMap.put(4.189, 47.0);

            shotHoodAngleMap.put(4.78, Rotation2d.fromDegrees(8));
            shotFlywheelSpeedMap.put(4.78, 47.0);
            timeOfFlightMap.put(4.78, 1.24);

            shotFlywheelSpeedMap.put(1.55, 33.0);
            shotHoodAngleMap.put(1.55,Rotation2d.fromDegrees(0));
            timeOfFlightMap.put(1.55,0.96);

            shotFlywheelSpeedMap.put(2.00,35.0);
            shotHoodAngleMap.put(2.00,Rotation2d.fromDegrees(0));
            timeOfFlightMap.put(2.00,0.92);




            //Alectron values
            // shotHoodAngleMap.put(2.6, Rotation2d.fromDegrees(18));
            // shotFlywheelSpeedMap.put(2.6, 60.0);
            // timeOfFlightMap.put(1.64, 0.93);

            // shotHoodAngleMap.put(2.02, Rotation2d.fromDegrees(15));
            // shotFlywheelSpeedMap.put(2.02, 60.0);
            // // timeOfFlightMap.put(1.64, 0.93);
            
            // shotHoodAngleMap.put(3.2, Rotation2d.fromDegrees(18.5));
            // shotFlywheelSpeedMap.put(3.2, 64.0);
            // // timeOfFlightMap.put(1.64, 0.93);

            // shotHoodAngleMap.put(1.8, Rotation2d.fromDegrees(13));
            // shotFlywheelSpeedMap.put(1.8, 60.0);
            // // timeOfFlightMap.put(1.64, 0.93);

            // shotHoodAngleMap.put(4.6, Rotation2d.fromDegrees(24.7));
            // shotFlywheelSpeedMap.put(4.6, 69.0);
            // // timeOfFlightMap.put(1.64, 0.93);

            // shotHoodAngleMap.put(2.75, Rotation2d.fromDegrees(21));
            // shotFlywheelSpeedMap.put(2.75, 51.0);

            passingShotHoodAngleMap.put(1.45, Rotation2d.fromDegrees(19.0)); //outlier??
            passingShotFlywheelSpeedMap.put(1.45, 20.0);
            passingTimeOfFlightMap.put(1.64, 0.93);

    }

    public record ShootParameters(
        double initialDistance,      // Static distance to target
        double lookaheadDistance,    // Distance to virtual "leading" target
        double hoodAngle,            // Degrees
        double flywheelSpeed,        // RPS
        double robotHeadingRadians,  // Field-relative rotation
        double hoodVelocity,         // Deg/Sec for motion profiling
        boolean isPassing,           // Whether we are in pass mode
        boolean isValid              // Safety check (is shot physically possible?)
    ) {}

    public shotCalculator(SwerveSubsystem drive) {
        this.drive = drive;
    }

    public static shotCalculator getInstance() {
        if (instance == null) {
            throw new RuntimeException("ShotCalculator must be initialized with a drivetrain first!");
        }
        return instance; 
    }

    public static shotCalculator getInstance(SwerveSubsystem drive) {
        if (instance == null) instance = new shotCalculator(drive);
        return instance;
    }

    @Override
    public void periodic() {    
        clearShootingParameters(); // Force recalculation every loop
        ShootParameters params = getParameters();
        
        // Dashboard Debugging
        SmartDashboard.putNumber("Shot Calculator/Distance", params.initialDistance);
        SmartDashboard.putNumber("Shot Calculator/Desired RPS", params.flywheelSpeed);
        SmartDashboard.putNumber("Shot Calculator/Desired Hood Angle", params.hoodAngle);
        SmartDashboard.putNumber("Shot Calculator/Desired Robot Heading", Units.radiansToDegrees(params.robotHeadingRadians));
        SmartDashboard.putNumber("Shot Calculator/Actual robot Heading", Units.radiansToDegrees(drive.getPose().getRotation().getDegrees()));


    }

    public ShootParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        Pose2d currentPose = drive.getPose();
        ChassisSpeeds fieldSpeeds = drive.getFieldVelocity();

        // 1. IMPROVED EXTRAPOLATION
        // Accounts for robot rotation during the phase delay
        Pose2d extrapolatedPose = currentPose.exp(
            new Twist2d(
                fieldSpeeds.vxMetersPerSecond * PHASE_DELAY_SEC,
                fieldSpeeds.vyMetersPerSecond * PHASE_DELAY_SEC,
                fieldSpeeds.omegaRadiansPerSecond * PHASE_DELAY_SEC
            )
        );

        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.orElse(Alliance.Blue) == Alliance.Red;
        double robotX = currentPose.getX();
        boolean isPastTrench = isRed ? (robotX < 11) : (robotX > 5.5);
        boolean isLeftRight = currentPose.getY() > 4;


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

        // 3. TANGENTIAL VELOCITY COMPENSATION (The Missing Link)
        // If the robot is rotating, the shooter (being offset from center) has extra velocity
        double robotRotation = extrapolatedPose.getRotation().getRadians();
        
        double tangentVelX = -fieldSpeeds.omegaRadiansPerSecond * shooterOffset.getY();
        double tangentVelY = fieldSpeeds.omegaRadiansPerSecond * shooterOffset.getX();
        
        // Rotate tangent velocity into field coordinates
        double totalVelX = fieldSpeeds.vxMetersPerSecond + (tangentVelX * Math.cos(robotRotation) - tangentVelY * Math.sin(robotRotation));
        double totalVelY = fieldSpeeds.vyMetersPerSecond + (tangentVelX * Math.sin(robotRotation) + tangentVelY * Math.cos(robotRotation));

        Pose2d shooterPosition = extrapolatedPose.transformBy(
            new Transform2d(shooterOffset, Constants.robotToShooter.getRotation().toRotation2d())
        );

        // 4. ITERATIVE LOOKAHEAD + HUB OFFSET
        double initialDistance = targetPose.getTranslation().getDistance(shooterPosition.getTranslation()) + Constants.ShooterConstants.hubOffset;
        double predictedTOF = currentTimeOfFlightMap.get(initialDistance);
        
        Translation2d virtualTarget = targetPose.getTranslation(); 
        for (int i = 0; i < 5; i++) {
            virtualTarget = targetPose.getTranslation().minus(
                new Translation2d(totalVelX * predictedTOF, totalVelY * predictedTOF)
            );
            double virtualDistance = virtualTarget.getDistance(shooterPosition.getTranslation()) + Constants.ShooterConstants.hubOffset;
            predictedTOF = currentTimeOfFlightMap.get(virtualDistance);
        }

        double lookaheadDistance = virtualTarget.getDistance(shooterPosition.getTranslation()) + Constants.ShooterConstants.hubOffset;

        // 5. LOOKUP & FILTERING
        double hoodAngle = currentShotHoodAngleMap.get(lookaheadDistance).getDegrees();
        double flywheelSpeed = currentShotFlywheelSpeedMap.get(lookaheadDistance);
        Rotation2d driveAngle = virtualTarget.minus(shooterPosition.getTranslation()).getAngle();

        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
        double filteredHoodVel = hoodVelocityFilter.calculate((hoodAngle - lastHoodAngle) / LOOP_PERIOD);
        lastHoodAngle = hoodAngle;

        latestParameters = new ShootParameters(
            initialDistance,
            lookaheadDistance,
            hoodAngle,
            flywheelSpeed,
            driveAngle.getRadians(),
            filteredHoodVel,
            false,
            lookaheadDistance > 1.0 && lookaheadDistance < 6.0
        );

        return latestParameters;
    }
    

    public void clearShootingParameters() {
        latestParameters = null;
    }
}