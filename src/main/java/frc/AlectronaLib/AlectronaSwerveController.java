package frc.AlectronaLib;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;

public class AlectronaSwerveController {
    private final PIDController translationController;
    private final PIDController rotationController;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final double invertDrive;
    private final double invertRot;
    private double distanceError = 30;

    // Added Limiters
    private final OPRSlewRateLimiter translationLimiter;
    private final OPRSlewRateLimiter autonTranslationLimiter;
    
    public AlectronaSwerveController(
        PIDController translationController, 
        PIDController rotationController, 
        double maxSpeed, 
        double maxAngularRate, 
        boolean invertDrive, 
        boolean invertRot,
        double driveRateLimit,   
        double driveJerkLimit,
        double autonRateLimit,
        double autonJerkLimit
    ) {
        this.rotationController = rotationController;
        this.translationController = translationController;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.invertDrive = invertDrive ? -1 : 1;
        this.invertRot = invertRot ? -1 : 1;
        this.translationLimiter = new OPRSlewRateLimiter(driveRateLimit, driveJerkLimit);
        this.autonTranslationLimiter = new OPRSlewRateLimiter(autonRateLimit, autonJerkLimit);
        this.translationController.calculate(Double.MAX_VALUE,0);
        this.rotationController.calculate(Double.MAX_VALUE, 0);
        this.rotationController.setTolerance(Units.degreesToRadians(3));
        rotationController.enableContinuousInput(0, 2 * Math.PI);

    }

    public Speeds calculate(Supplier<Pose2d> currentPoseInput, Supplier<Pose2d> targetPoseInput, DoubleSupplier XTranslationInput, DoubleSupplier YTranslationInput, Supplier<Rotation2d> targetRotationInput, DoubleSupplier rightX, DoubleSupplier velocityInput, boolean clampRotation){ 
        double vx, vy, vr;
        Pose2d currentPose, target;
        Rotation2d targetRotation;
        double rawX = 0;
        double rawY = 0;
        if (YTranslationInput != null || XTranslationInput != null) {
            rawY = YTranslationInput.getAsDouble() * -1;
            rawX = XTranslationInput.getAsDouble() * -1;
            double magnitude = Math.hypot(rawX, rawY);
            if (magnitude > 1.0) {
                rawX /= magnitude;
                rawY /= magnitude;
            }   
        }
        currentPose = currentPoseInput.get();
        target = (targetPoseInput == null ? null:targetPoseInput.get());
        targetRotation = (targetRotationInput == null ? null:targetRotationInput.get());
        if (target != null) {
            double velocity;
            // drive to pose, override pose target rot if target rotation exists
            double distance = currentPose.getTranslation().getDistance(target.getTranslation());
            if (velocityInput != null){
                velocity = velocityInput.getAsDouble();
                distanceError = distance;
            } else{
                velocity = translationController.calculate(distance, 0) * -1;
                distanceError = translationController.getPositionError();
            }
            Rotation2d angleToTarget = target.getTranslation().minus(currentPose.getTranslation()).getAngle();
            
            vx = Math.cos(angleToTarget.getRadians()) * velocity * invertDrive;
            vy = Math.sin(angleToTarget.getRadians()) * velocity * invertDrive;
            
            Rotation2d rotGoal = (targetRotation != null) ? targetRotation : target.getRotation();
            vr = rotationController.calculate(currentPose.getRotation().getRadians(), rotGoal.getRadians()) * invertRot;

        } else if (targetRotation != null) {
            // Drive w angle
            vx = rawY * maxSpeed;
            vy = rawX * maxSpeed;
            vr = rotationController.calculate(currentPose.getRotation().getRadians(), targetRotation.getRadians()) * invertRot;
            
        } else {
            // manual Drive
            vx = rawY * maxSpeed;
            vy = rawX * maxSpeed;
            if (rightX != null){
                vr = rightX.getAsDouble() * maxAngularRate * -1;
            } else {
                vr = 0;
            }
        }

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
        final double finalVr = clampRotation ? MathUtil.clamp(vr,-2*Math.PI,2*Math.PI) : vr;

        return new Speeds(() -> finalVx, () -> finalVy, () -> finalVr);
    }

    public double getDistanceError() {
        return Math.abs(this.distanceError);
    }

    public double getRotationalError() {
        return Math.abs(rotationController.getPositionError());
    }

    public record Speeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr) {}
}