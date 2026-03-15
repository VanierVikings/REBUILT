package frc.AlectronaLib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Set;

public class EstimatePose {
    // Field dimensions in meters
    private static final double kFieldLength = 16.6;
    private static final double kFieldWidth = 8.1;
    // Hub tags we trust more
    private static final Set<Integer> PRIORITY_TAGS = Set.of(
        8, 5, 9, 10, 4, 3, 11, 2, 18, 27, 19, 20, 26, 25, 21, 24
    );

    private static final double kMaxAmbiguity = 0.7;
    
    // Distance Thresholds
    private static final double kMaxDistTrusted = 6.0; // 6m for priority tags
    private static final double kMaxDistNormal = 3.0;  // 3m for non-priority tags

    private final String m_name;
    private double m_lastTagTimestamp = 0;

    public static record RobotState(
        edu.wpi.first.math.kinematics.ChassisSpeeds Speeds,
        edu.wpi.first.math.geometry.Pose2d Pose
    ) {}

    public EstimatePose(String name) {
        this.m_name = name;
        this.m_lastTagTimestamp = Timer.getFPGATimestamp();
        LimelightHelpers.setCameraPose_RobotSpace(m_name, Units.inchesToMeters(12.054), Units.inchesToMeters(5.967579), Units.inchesToMeters(17.431841), 0, 20.7836, 0); //herhejihejlrafna
        LimelightHelpers.SetIMUMode(m_name, 0); 
    }

    /**
     * Checks if the estimated pose is physically located within the field boundaries.
     */
    private boolean isPoseInField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= 0 && x <= kFieldLength && y >= 0 && y <= kFieldWidth;
    }

    public void update(RobotState robotState, VisionUpdateConsumer visionConsumer) {
        // megaTag 1 logic - Disabled/Auto only
        if (DriverStation.isDisabled() || DriverStation.isAutonomous()) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
            
            if (mt1 != null && mt1.tagCount > 0) {
                Matrix<N3, N1> stdDevs = calculateMT1StdDevs(mt1);
                
                if (stdDevs.get(0, 0) != Double.MAX_VALUE) {
                    if (isPoseInField(mt1.pose)) {
                        m_lastTagTimestamp = Timer.getFPGATimestamp(); 
                        visionConsumer.accept(mt1.pose, mt1.timestampSeconds, stdDevs);
                    }
                }
            }
        }
    LimelightHelpers.SetRobotOrientation(m_name, robotState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // MegaTag 2 Logic
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);

        if (mt2 != null && mt2.tagCount > 0) {
            double omegaDegrees = Math.abs(Units.radiansToDegrees(robotState.Speeds.omegaRadiansPerSecond));
            if (omegaDegrees < 50) { 
                Matrix<N3, N1> stdDevs = calculateMT2StdDevs(mt2, omegaDegrees);
                
                if (stdDevs.get(0, 0) != Double.MAX_VALUE) {
                    if (isPoseInField(mt2.pose)) {
                        m_lastTagTimestamp = Timer.getFPGATimestamp();
                        visionConsumer.accept(mt2.pose, mt2.timestampSeconds, stdDevs);
                    }
                }
            }
        }
    }

    private Matrix<N3, N1> calculateMT1StdDevs(LimelightHelpers.PoseEstimate estimate) {
        double avgAmbiguity = 0;
        double avgDist = 0;
        int count = estimate.rawFiducials.length;
        boolean seesPriorityTag = false;

        for (var tag : estimate.rawFiducials) {
            avgAmbiguity += tag.ambiguity;
            avgDist += tag.distToCamera;
            if (PRIORITY_TAGS.contains(tag.id)) seesPriorityTag = true;
        }
        
        if (count > 0) {
            avgAmbiguity /= count;
            avgDist /= count;
        }

        // apply distance filter based on priority status
        double maxAllowedDist = seesPriorityTag ? kMaxDistTrusted : kMaxDistNormal;
        
        if (avgAmbiguity > kMaxAmbiguity) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        if (count == 1 && avgDist > maxAllowedDist) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        double xyStdDev = (count > 1) ? 0.3 : 0.6;
        xyStdDev *= (1 + (Math.pow(avgDist, 2) / 30.0));

        // priority Weighting
        if (seesPriorityTag) {
            xyStdDev *= 0.6; 
        }
        
        return VecBuilder.fill(xyStdDev, xyStdDev, 0.7); 
    }

    private Matrix<N3, N1> calculateMT2StdDevs(LimelightHelpers.PoseEstimate estimate, double omegaDegrees) {
    double avgDist = 0;
    int count = estimate.rawFiducials.length;
    boolean seesPriorityTag = false;

    for (var tag : estimate.rawFiducials) {
        avgDist += tag.distToCamera;
        if (PRIORITY_TAGS.contains(tag.id)) {
            seesPriorityTag = true;
        }
    }
    
    if (count > 0) avgDist /= count;

    // Apply distance filter based on priority status
    double maxAllowedDist = seesPriorityTag ? kMaxDistTrusted : kMaxDistNormal;
    if (count == 1 && avgDist > maxAllowedDist) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    // --- ADJUSTED BASE TRUST ---
    // If count > 1, base is 1.0. If count == 1, base is 1.5.
    double xyStdDev = (count > 1) ? 1.0 : 1.5;
    
    // Distance scaling (same logic as before)
    xyStdDev *= (1 + (Math.pow(avgDist, 2) / 15.0));

    // If > 360 return MAX_VALUE
    if (omegaDegrees > 360) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    // Scaling factor for rotation
    double rotationMultiplier = 1.0 + (omegaDegrees - 50.0) * (15.0 / 310.0); 
    xyStdDev *= rotationMultiplier;

    // --- ADJUSTED PRIORITY WEIGHT ---
    // 1.0 (Base) * 0.6 (Priority) = 0.6 Total
    if (seesPriorityTag) {
        xyStdDev *= 0.6; 
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);
}

    public double getLastTagTimestamp() { return m_lastTagTimestamp; }

    @FunctionalInterface
    public interface VisionUpdateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }
}