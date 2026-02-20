package frc.AlectronaLib;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class AlectronaSwerveController {
    private final PIDController translationController;
    private final PIDController rotationController;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final double invertDrive;
    private final double invertRot;

    // Added Limiters
    private final OPRSlewRateLimiter translationLimiter;
    
    public AlectronaSwerveController(
        PIDController translationController, 
        PIDController rotationController, 
        double maxSpeed, 
        double maxAngularRate, 
        boolean invertDrive, 
        boolean invertRot,
        double driveRateLimit,   
        double driveJerkLimit
    ) {
        this.rotationController = rotationController;
        this.translationController = translationController;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.invertDrive = invertDrive ? -1 : 1;
        this.invertRot = invertRot ? -1 : 1;
        this.translationLimiter = new OPRSlewRateLimiter(driveRateLimit, driveJerkLimit);
        this.translationController.calculate(Double.MAX_VALUE,0);
        this.rotationController.calculate(Double.MAX_VALUE, 0);
        this.rotationController.setTolerance(Units.degreesToRadians(3));
        rotationController.enableContinuousInput(0, 2 * Math.PI);

    }

    public Speeds calculate(Supplier<Pose2d> currentPoseInput, Supplier<Pose2d> targetPoseInput, DoubleSupplier XTranslationInput, DoubleSupplier YTranslationInput, Supplier<Rotation2d> targetRotationInput, DoubleSupplier rightX) {
        double vx, vy, vr;
        Pose2d currentPose, target;
        Rotation2d targetRotation;  
        double rawY = YTranslationInput.getAsDouble() * -1;
        double rawX = XTranslationInput.getAsDouble() * -1;
        double magnitude = Math.hypot(rawX, rawY);
        if (magnitude > 1.0) {
            rawX /= magnitude;
            rawY /= magnitude;
        }
        currentPose = currentPoseInput.get();
        target = (targetPoseInput == null ? null:targetPoseInput.get());
        targetRotation = (targetRotationInput == null ? null:targetRotationInput.get());

        if (target != null) {
            // drive to pose, override pose target rot if target rotation exists
            double distance = currentPose.getTranslation().getDistance(target.getTranslation());
            double velocity = translationController.calculate(distance, 0) * -1;
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
            vr = rightX.getAsDouble() * maxAngularRate * -1;
        }

        double rawMag = Math.sqrt(vx * vx + vy * vy);
        if (rawMag > 1e-6) {
            double limitedMag = translationLimiter.calculate(rawMag);
            double ratio = limitedMag / rawMag;
            vx *= ratio;
            vy *= ratio;
        } else {
            translationLimiter.reset(0);
        }

        final double finalVx = vx;
        final double finalVy = vy;
        final double finalVr = vr;

        return new Speeds(() -> finalVx, () -> finalVy, () -> finalVr);
    }

    public double getDistanceError() {
        return Math.abs(translationController.getPositionError());
    }

    public double getRotationalError() {
        return Math.abs(rotationController.getPositionError());
    }

    public record Speeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr) {}
}