package frc.AlectronaLib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import java.util.function.Supplier;


public class EstimatePose {
    private final String m_name;
    private double m_lastTagTimestamp = 0;

    public static record RobotState(
        edu.wpi.first.math.kinematics.ChassisSpeeds Speeds,
        edu.wpi.first.math.geometry.Pose2d Pose
    ) {}

    /**
     * @param name 
     */
    public EstimatePose(String name) {
        this.m_name = name;
        this.m_lastTagTimestamp = Timer.getFPGATimestamp();
        
        // Initial Camera Offset Calibration
        LimelightHelpers.setCameraPose_RobotSpace(m_name, 0.33401, 0, 0.45578, 0, 25, 0);
    }

    /**
     * Processes vision updates. 
     * @param robotState Current state of the drivetrain
     * @param visionConsumer Function to pass the Pose, Timestamp, and StdDevs back to the PoseEstimator
     */
    public void update(RobotState robotState, VisionUpdateConsumer visionConsumer) {
        
        if (DriverStation.isDisabled() || DriverStation.isAutonomous()) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
            if (mt1 != null && LimelightHelpers.getTV(m_name)) {
                m_lastTagTimestamp = Timer.getFPGATimestamp(); 
                
                visionConsumer.accept(
                    mt1.pose,
                    mt1.timestampSeconds,
                    VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 0.7)
                );
            }
        }

        boolean isStationary = Math.abs(robotState.Speeds.vxMetersPerSecond) < 1
                            && Math.abs(robotState.Speeds.vyMetersPerSecond) < 1
                            && Math.abs(robotState.Speeds.omegaRadiansPerSecond) < Math.PI;

        LimelightHelpers.SetIMUMode(m_name, isStationary ? 1 : 2);

        LimelightHelpers.SetRobotOrientation(
            m_name,
            robotState.Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);
    
        if (mt2 != null && mt2.tagCount > 0) {
            boolean rotatingTooFast = Math.abs(Units.radiansToDegrees(robotState.Speeds.omegaRadiansPerSecond)) > 360;

            if (!rotatingTooFast) {
                m_lastTagTimestamp = Timer.getFPGATimestamp();
                visionConsumer.accept(
                    mt2.pose, 
                    mt2.timestampSeconds, 
                    VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE)
                );
            }
        }
    }

    public double getLastTagTimestamp() {
        return m_lastTagTimestamp;
    }

    /** Functional interface for passing vision data back to the Swerve Drive */
    @FunctionalInterface
    public interface VisionUpdateConsumer {
        void accept(edu.wpi.first.math.geometry.Pose2d pose, double timestamp, edu.wpi.first.math.Vector<edu.wpi.first.math.numbers.N3> stdDevs);
    }
}