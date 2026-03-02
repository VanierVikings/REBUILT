package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.SpindexerConstants; 

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class spindexerSubsystem extends SubsystemBase {

    private SparkMax spindexerMotor;
    private SparkMaxConfig spindexerConfig;
    SparkClosedLoopController spindexController;
   
    public spindexerSubsystem() {
        spindexerMotor = new SparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID,MotorType.kBrushless);
        
        spindexerConfig = new SparkMaxConfig(); 
        spindexerConfig
            .smartCurrentLimit(SpindexerConstants.SPINDEXER_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(10);


        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void runSpindexer(){
        spindexerMotor.setVoltage(10); //voltage control yaay
    }

    public void stopSpindexer(){
        spindexerMotor.stopMotor();
    }


    public Command setState(SuperStructure.SpindexerStates state){
        Command command;
        switch (state) {
            case FEED:
                command = runOnce(()->{
                    runSpindexer();
                });
                break;

            case OFF:
                command = runOnce(()->{
                    stopSpindexer();
                });
                break;


            default:
                command = runOnce(()->{
                    stopSpindexer();
                });
                break;
        }
        return command;
    }

}

