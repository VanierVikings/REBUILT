package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;



public class shooter extends SubsystemBase {
    private final TalonFX shooterMotorTop;
    private final TalonFX shooterMotorBack;
    private final SparkMax shooterMotorBottom;
    private final TalonFXConfiguration shooterConfig;


    

    public shooter(){
        shooterMotorTop = new TalonFX(ShooterConstants.shooterMotorTopMasterID);
        shooterMotorBack = new TalonFX(ShooterConstants.shooterMotorTopSlaveID);
        shooterMotorBottom = new SparkMax(ShooterConstants.shooterMotorBottomID, SparkMax.MotorType.kBrushless);

        shooterMotorBack.setControl(new Follower(ShooterConstants.shooterMotorTopMasterID, MotorAlignmentValue.Opposed));

        shooterConfig = new TalonFXConfiguration();
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;


        



        

        
    }
}
