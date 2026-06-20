package frc.VikingsLib;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveController {
    private final PIDController translationController;
    private final PIDController rotationController;
    private final double invertDrive;
    private final double invertRot;
    private double distanceError = 30;

    // Added Limiters
    private final OPRSlewRateLimiter translationLimiter;
    private final OPRSlewRateLimiter autonTranslationLimiter;
    
    public SwerveController(
        PIDController translationController, 
        PIDController rotationController, 
        boolean invertDrive, 
        boolean invertRot,
        double driveRateLimit,   
        double driveJerkLimit,
        double autonRateLimit,
        double autonJerkLimit
    ) {
        this.rotationController = rotationController;
        this.translationController = translationController;
        this.invertDrive = invertDrive ? -1 : 1;
        this.invertRot = invertRot ? -1 : 1;
        this.translationLimiter = new OPRSlewRateLimiter(driveRateLimit, driveJerkLimit);
        this.autonTranslationLimiter = new OPRSlewRateLimiter(autonRateLimit, autonJerkLimit);
        this.translationController.calculate(Double.MAX_VALUE,0);
        this.rotationController.calculate(Double.MAX_VALUE, 0);
        this.rotationController.setTolerance(Units.degreesToRadians(3));
        rotationController.enableContinuousInput(0, 2 * Math.PI);
    }

    public Speeds calculate(Supplier<Pose2d> currentPoseInput, Supplier<Pose2d> targetPoseInput){ 
        double vx, vy, vr;
        Pose2d currentPose, targetPose;
        currentPose = currentPoseInput.get();
        targetPose = targetPoseInput.get(); 
        // drive to pose, override pose target rot if target rotation exists
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double velocity = translationController.calculate(distance, 0) * -1;
        distanceError = translationController.getPositionError();
        Rotation2d angleToTarget = targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        
        vx = Math.cos(angleToTarget.getRadians()) * velocity * invertDrive;
        vy = Math.sin(angleToTarget.getRadians()) * velocity * invertDrive;  
        vr = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()) * invertRot;

        double rawMag = Math.sqrt(vx * vx + vy * vy);
        if (rawMag > 1e-6) {
            double limitedMag;
            if (DriverStation.isAutonomous()) {
                limitedMag = autonTranslationLimiter.calculate(rawMag);
            } else {
                limitedMag = translationLimiter.calculate(rawMag);
            }
            double ratio = limitedMag / rawMag;
            vx *= ratio;
            vy *= ratio;
        } else {
            translationLimiter.reset(0);
        }
        
        final double finalVx = vx;
        final double finalVy = vy;
        final double finalVr = MathUtil.clamp(vr,-2*Math.PI,2*Math.PI);
        // SmartDashboard.putNumber("DT Speed / Vx", finalVx);
        // SmartDashboard.putNumber("DT Speed / Vy", finalVy);
        // SmartDashboard.putNumber("DT Speed / Vr", finalVr);

        return new Speeds(finalVx, finalVy, finalVr);
    }

        public double getDistanceError() {
        return Math.abs(this.distanceError);
    }

    public double getRotationalError() {
        return Math.abs(rotationController.getPositionError());
    }


    public record Speeds(Double vx, Double vy, Double vr) {}
}