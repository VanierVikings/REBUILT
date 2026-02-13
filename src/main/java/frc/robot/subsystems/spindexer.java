package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;



public class spindexer extends SubsystemBase {

    private SparkMax mSparkMax = new SparkMax(0, MotorType.kBrushless); 
    private AbsoluteEncoder mEncoder; 

    public spindexer() {
       spindexerConfig();   
    }

    public void spindexerConfig() {
         SparkMaxConfig mConfig = new SparkMaxConfig(); 
    //just set it to 0, will probably tune later 
    SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(0, 0,0); 
     //will change in future
      mConfig.smartCurrentLimit(30);
      mSparkMax.configure(mConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 
    
    }


}
