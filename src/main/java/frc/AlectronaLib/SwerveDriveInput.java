package frc.AlectronaLib;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;

public class SwerveDriveInput {

    public SwerveDriveInput() {
    }

    public Translation2d getShapedInput(DoubleSupplier x, DoubleSupplier y) {
        double xInput = x.getAsDouble();
        double yInput = y.getAsDouble();
        double rawMagnitude = Math.sqrt(xInput * xInput + yInput * yInput);

        // Apply deadband
        if (rawMagnitude < DriveConstants.deadband) {
            return new Translation2d(0, 0);
        }

        // Remap magnitude to 0-1 range after deadband
        double remappedMagnitude = Math.min(1.0, (rawMagnitude - DriveConstants.deadband) / (1.0 - DriveConstants.deadband));
        
        // Apply sine-based curve for smoother low-speed control
        double shapedMagnitude = Math.pow(Math.sin((Math.PI / 2.0) * remappedMagnitude), 2);

        // Calculate angle and return translation without rate limiting
        double angle = Math.atan2(yInput, xInput);
        if (DriverStation.isEnabled()){
            return new Translation2d(
            shapedMagnitude * Math.cos(angle),
            shapedMagnitude * Math.sin(angle)
        );
        } else{
            return new Translation2d(0,0);
        }
        
    }
}