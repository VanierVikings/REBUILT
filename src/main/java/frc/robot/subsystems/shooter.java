package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    private final SparkMax shooterTopSpinMotor;
    private final SparkMax shooterCenterMotor;
    private final SparkMax feederMotor;
    private final TalonFX hoodMotor;
    private final SparkMaxConfig shooterConfig;

    // private final VelocityVoltage m_request;
    // private final InterpolatingDoubleTreeMap rpmMap;
    // private final InterpolatingDoubleTreeMap angleMap;
    private final TalonFXConfiguration hoodMotorConfig;
    // private final RelativeEncoder hoodEncoder;
    // private final SparkClosedLoopController hoodClosedLoopController;


    public shooter(){
        shooterCenterMotor = new SparkMax(ShooterConstants.shooterCenterMotorID, MotorType.kBrushless);
        shooterTopSpinMotor = new SparkMax(ShooterConstants.shooterTopSpinMotorID,MotorType.kBrushless);
        feederMotor = new SparkMax(ShooterConstants.feederMotorID, MotorType.kBrushless);
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorID);

        shooterConfig = new SparkMaxConfig();
        hoodMotorConfig = new TalonFXConfiguration();



        shooterConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);        
            
        shooterTopSpinMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterCenterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




        

    }

}
