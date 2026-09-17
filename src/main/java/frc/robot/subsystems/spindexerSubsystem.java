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
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class spindexerSubsystem extends SubsystemBase {

    private SparkMax spindexerMotor;
    private SparkMaxConfig spindexerConfig;
    SparkClosedLoopController spindexController;
    private final SparkMaxSim spindexerMotorSim;
    private SuperStructure.SpindexerStates requestedState = SuperStructure.SpindexerStates.OFF;
    private double appliedVoltage;
   
    public spindexerSubsystem() {
        spindexerMotor = new SparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID,MotorType.kBrushless);
        spindexerMotorSim = new SparkMaxSim(spindexerMotor, DCMotor.getNEO(1));
        
        spindexerConfig = new SparkMaxConfig(); 
        spindexerConfig
            .smartCurrentLimit(SpindexerConstants.SPINDEXER_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(30);

        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void runSpindexer(double voltage){
        appliedVoltage = voltage;
        spindexerMotor.setVoltage(voltage); //voltage control yaay
    }

    public void stopSpindexer(){
        appliedVoltage = 0;
        spindexerMotor.setVoltage(0);
        spindexerMotor.stopMotor();
    }

    public Command runEndSpindexer(){
        return this.runEnd(()->runSpindexer(7), ()->stopSpindexer());
    }


    public Command setState(SuperStructure.SpindexerStates state){
        return runOnce(() -> applyState(state));
    }

    public void applyState(SuperStructure.SpindexerStates state) {
        requestedState = state;
        switch (state) {
            case FEED -> runSpindexer(7);
            case SLOW -> runSpindexer(1);
            case JAM -> runSpindexer(-7);
            default -> stopSpindexer();
        }
    }

    public double getAppliedVoltage() { return appliedVoltage; }
    public SuperStructure.SpindexerStates getRequestedState() { return requestedState; }

    @Override
    public void periodic() {
        SmartDashboard.putString("Spindexer/RequestedState", requestedState.toString());
        SmartDashboard.putNumber("Spindexer/AppliedVoltage", appliedVoltage);
    }

    @Override
    public void simulationPeriodic() {
        spindexerMotorSim.iterate(appliedVoltage == 0 ? 0 : Math.copySign(3000, appliedVoltage), 12.0, 0.02);
    }

}

