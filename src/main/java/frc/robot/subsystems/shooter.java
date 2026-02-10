package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.controls.Follower;



public class shooter extends SubsystemBase {
    private final TalonFX shooterMotorTopMaster;
    private final TalonFX shooterMotorTopSlave;
    private final SparkMax shooterMotorBottom;
    

    public shooter(){
        shooterMotorTopMaster = new TalonFX(ShooterConstants.shooterMotorTopMasterID);
        shooterMotorTopSlave = new TalonFX(ShooterConstants.shooterMotorTopSlaveID);
        shooterMotorBottom = new SparkMax(ShooterConstants.shooterMotorBottomID, SparkMax.MotorType.kBrushless);

        shooterMotorTopSlave.setControl(new Follower(ShooterConstants.shooterMotorTopMasterID, MotorAlignmentValue.Opposed));



        

        
    }
}
