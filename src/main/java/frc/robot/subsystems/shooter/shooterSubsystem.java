package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperStructure.ShooterStates;
import frc.robot.subsystems.SuperStructure;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



public class shooterSubsystem extends SubsystemBase{
    private final TalonFX shooterLeaderMotor;
    private final TalonFX shooterFollowerMotor;
    private final TalonFX hoodMotor;
    private final SparkMax feederMotor;

    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration hoodConfig;
    private final SparkMaxConfig feederConfig;


    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);
    
    private ShooterStates currentState;

    public shooterSubsystem(){
        currentState = ShooterStates.HOME;
        shooterLeaderMotor = new TalonFX(ShooterConstants.shooterLeaderMotor);
        shooterFollowerMotor = new TalonFX(ShooterConstants.shooterFollowerMotor);
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorID);
        feederMotor = new SparkMax(ShooterConstants.feederMotorID, MotorType.kBrushless);

        /* FLYWHEEL CONFIGS */
        flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 100;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = 2;
        // flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;

        flywheelConfig.Slot0.kS = 0.26;
        flywheelConfig.Slot0.kV = 0.12;
        flywheelConfig.Slot0.kP = 0.1;

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterLeaderMotor.getConfigurator().apply(flywheelConfig);
        shooterFollowerMotor.setControl(new Follower(ShooterConstants.shooterLeaderMotor, MotorAlignmentValue.Opposed));




        /* HOOD CONFIGS */
        hoodConfig = new TalonFXConfiguration();

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        hoodConfig.MotionMagic.MotionMagicAcceleration = 3.0;

        hoodConfig.Feedback.SensorToMechanismRatio = (4.0*(365.0/30.0)); // 4:1 maxplanetary + 365:30 rack and pinion
        hoodConfig.Slot0.kS = 0.2;
        hoodConfig.Slot0.kV = (0.12*(4.0*(365.0/30.0)));
        hoodConfig.Slot0.kP = 200;
        hoodConfig.Slot0.kD = 0;

        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodMotor.setPosition(0);
    
        // //limit stuff



        /* FEEDER CONFIGS */
        feederConfig = new SparkMaxConfig();
        feederConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        





    } 

    public void setShooterRPS(double rps){
        shooterLeaderMotor.setControl(m_request.withVelocity(rps));
 }

    public void setFeederVoltage(double voltage){
        feederMotor.setVoltage(voltage);
    }

    public void setHoodAngle(double targetDegrees){
        targetDegrees = Units.degreesToRotations(targetDegrees);
        hoodMotor.setControl(m_motionMagic.withPosition(targetDegrees));
    }

    public void stopShooter(){
        shooterLeaderMotor.stopMotor();
    }

    public void stopFeeder(){
        feederMotor.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter/Shooter RPS", shooterLeaderMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Hood Angle (degrees)", hoodMotor.getPosition().getValueAsDouble()*360);
        SmartDashboard.putString("Shooter/Shooter State", this.currentState.toString());
        SmartDashboard.putNumber("Current Hood Pos", (hoodMotor.getPosition().getValueAsDouble())*360.0);

        SmartDashboard.putNumber("Shooter Inputs/Input Shooter RPS", 20);

    }


    public Command setState(SuperStructure.ShooterStates state){
        this.currentState = state;
        Command command;
        switch (state) {
            case TEST:
                command = run(()->{
                    double targetRPS = SmartDashboard.getNumber("Shooter Inputs/Input Shooter RPS",0 );
                    double targetAngle = SmartDashboard.getNumber("Shooter Inputs/Input Hood Angle", 0);
                    boolean shouldFeed = SmartDashboard.getBoolean("Shooter Inputs/Enable Feeder", false);
                    setHoodAngle(targetRPS);
                    setShooterRPS(targetAngle);
                    setFeederVoltage(shouldFeed ? 5: 0);
                });
                break;
        
            case IDLE:
                command = run(()->{
                    setHoodAngle(20);
                    stopFeeder();
                    stopShooter();
                });
                break;

            default:
            command = run(()->{
                setHoodAngle(10);
                stopShooter();
                stopFeeder();
            });
                break;
        };
        return command;
    }
}
