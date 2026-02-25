package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.MathUtil;



public class shooter extends SubsystemBase {
    private final SparkMax shooterLeftMotor;
    private final SparkMax shooterRightMotor;
    private final SparkMax feederMotor;
    private final TalonFX hoodMotor;
    private final SparkMaxConfig rightFlyWheelConfig;
    private final SparkMaxConfig leftFlywheelConfig;

    // private final VelocityVoltage m_request;
    // private final InterpolatingDoubleTreeMap rpmMap;
    // private final InterpolatingDoubleTreeMap angleMap;
    private final TalonFXConfiguration hoodMotorConfig;
    // private final RelativeEncoder hoodEncoder;
    // private final SparkClosedLoopController hoodClosedLoopController;
    
    private final SparkClosedLoopController rightFlyWheelController;


    public shooter(){
        shooterRightMotor = new SparkMax(ShooterConstants.shooterCenterMotorID, MotorType.kBrushless);
        shooterLeftMotor = new SparkMax(ShooterConstants.shooterTopSpinMotorID,MotorType.kBrushless);
        feederMotor = new SparkMax(ShooterConstants.feederMotorID, MotorType.kBrushless);
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorID);

        rightFlyWheelConfig = new SparkMaxConfig();
        leftFlywheelConfig = new SparkMaxConfig();

        hoodMotorConfig = new TalonFXConfiguration();


        rightFlyWheelController = shooterRightMotor.getClosedLoopController();


        /*
        * Flywheel configs
        */
        rightFlyWheelConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .closedLoop
                .pid(0, 0, 0) //recalc it
                .feedForward
                    .kS(0) //recalc it
                    .kV(0)
                    .kA(0);

        leftFlywheelConfig
            .apply(rightFlyWheelConfig)
            .follow(shooterRightMotor)
            .inverted(true);
            
        shooterRightMotor.configure(rightFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterLeftMotor.configure(leftFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor.configure(rightFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        

        /*
         * Hood Configs
         */
        // hoodMotorConfig.Feedback.SensorToMechanismRatio = () NEED TO ADD RATIO
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit= 40;
        hoodMotorConfig.Slot0.kS = 0.0; //recalc it
        hoodMotorConfig.Slot0.kV = 0.0;
        hoodMotorConfig.Slot0.kA = 0.0;
        hoodMotorConfig.Slot0.kP = 0.0;
        hoodMotorConfig.Slot0.kI = 0.0;
        hoodMotorConfig.Slot0.kD = 0.0;

        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            hoodMotor.getConfigurator().apply(hoodMotorConfig);
        }
    
        public void runFeeder(double direction){
            feederMotor.set(direction);
        }

        public void stopFeeder(){
            feederMotor.set(0);
        }

        public void setShooterSpeeds(double rps){
            rightFlyWheelController.setSetpoint(rps, ControlType.kVelocity);
        }

        public void stopShooter(){
            rightFlyWheelController.setSetpoint(0, ControlType.kVelocity);
        }


        @Override
        public void periodic(){
            SmartDashboard.putNumber("Shooter RPS", shooterRightMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Hood Angle degrees", hoodMotor.getPosition().getValueAsDouble());
            
        }




        

}
