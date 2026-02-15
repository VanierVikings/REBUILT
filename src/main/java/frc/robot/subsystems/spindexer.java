package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.SpindexerConstants; 

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class spindexer extends SubsystemBase {

    private SparkMax spindexerMotor;
    private SparkMaxConfig spindexerConfig;
    SparkClosedLoopController spindexController;
   
    public spindexer() {
        spindexerMotor = new SparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID,MotorType.kBrushless);

        spindexerConfig = new SparkMaxConfig(); 
        spindexerConfig
        .smartCurrentLimit(SpindexerConstants.SPINDEXER_CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast);

        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public Command spin(double direction){
            return this.runEnd(() -> spindexerMotor.set(direction), () -> spindexerMotor.set(0));
        }


}
