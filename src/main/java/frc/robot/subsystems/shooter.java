package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;



public class shooter extends SubsystemBase {
    private final TalonFX shooterMotorTop;
    private final TalonFX shooterMotorBack;
    private final SparkMax shooterMotorBottom;
    private final TalonFXConfiguration shooterConfig;
    private final SparkMaxConfig bottomConfig;


    

    public shooter(){
        shooterMotorTop = new TalonFX(ShooterConstants.shooterMotorTopMasterID);
        shooterMotorBack = new TalonFX(ShooterConstants.shooterMotorTopSlaveID);
        shooterMotorBottom = new SparkMax(ShooterConstants.shooterMotorBottomID, SparkMax.MotorType.kBrushless);

        shooterMotorBack.setControl(new Follower(ShooterConstants.shooterMotorTopMasterID, MotorAlignmentValue.Opposed));

        shooterConfig = new TalonFXConfiguration();
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TO BE CHANGED

        bottomConfig = new SparkMaxConfig();
        bottomConfig
        .smartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);


    }

    public void spinTop(double direciton){
        shooterMotorTop.setControl(new DutyCycleOut(direciton));
    }

    public Command spinBottom(double direction){
            return this.runEnd(()-> shooterMotorBottom.set(direction), () -> shooterMotorBottom.set(0));
        }



}
