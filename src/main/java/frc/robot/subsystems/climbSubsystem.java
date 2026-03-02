package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.SuperStructure.CLimberStates;


public class climbSubsystem extends SubsystemBase {
    private final SparkMax climbMotor;
    private final SparkClosedLoopController climbController;
    private final SparkMaxConfig climbConfig;
    private final RelativeEncoder climbEncoder;
    private final Debouncer m_currentDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private CLimberStates currentState;
    
        public climbSubsystem(){
            currentState = CLimberStates.HOME;
            climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
            climbController = climbMotor.getClosedLoopController();
            climbConfig = new SparkMaxConfig();
            climbEncoder = climbMotor.getEncoder();
    
    
    
    
            climbConfig
                .smartCurrentLimit(45)
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .closedLoop
                    .p(ClimbConstants.kP)
                    .i(ClimbConstants.kI)
                    .d(ClimbConstants.kD);
    
            
                climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            SparkClosedLoopController climbController = climbMotor.getClosedLoopController();
    
            
                // .outputRange(kMinOutput, kMaxOutput);
        }
    
        @Override
        public void periodic(){
            SmartDashboard.putNumber("Climb position",climbEncoder.getPosition());
        }
    
        public void setSoftLimits(boolean active){
            climbConfig
                .softLimit
                    .forwardSoftLimitEnabled(active)
                    .reverseSoftLimitEnabled(false);
            climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    
        public void setClimbPosition(double position){
            climbController.setSetpoint(position, ControlType.kMAXMotionPositionControl); //rotations
        }
    
        public void resetClimbEncoder(){
            climbEncoder.setPosition(0); //rotations
        }

        public void stopClimbMotor(){
            climbMotor.stopMotor();
        }
    
    
        public Command setState(SuperStructure.CLimberStates state){
            this.currentState = state;
            Command command;
            switch (state) {
                case TEST:
                    command = run(()->{
                        Double testPos = SmartDashboard.getNumber("Climber test position (rotations)", 0.0);
                        setClimbPosition(testPos);
                    });
                    break;
            
                default:
                command = run(()->{
                    setClimbPosition(0);
                    stopClimbMotor();
                });
                    break;
            }
            return command;



    }

    
}
