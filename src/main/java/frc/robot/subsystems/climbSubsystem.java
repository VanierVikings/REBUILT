package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class climbSubsystem extends SubsystemBase {
    private final SparkMax climbMotor;
    private final SparkClosedLoopController climbController;
    private final SparkMaxConfig climbConfig;

    public climbSubsystem(){
        climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
        climbController = climbMotor.getClosedLoopController();
        climbConfig = new SparkMaxConfig();


        climbConfig
            .smartCurrentLimit(45)
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    
    
}
