package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



public class shooterSubsystem extends SubsystemBase{
    public final TalonFX shooterLeaderMotor;
    private final TalonFX shooterFollowerMotor;
    public final TalonFX hoodMotor;
    private final SparkMax feederMotor;

    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration hoodConfig;
    private final SparkMaxConfig feederConfig;


    public final VelocityVoltage m_request = new VelocityVoltage(0);
    public final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);
    private final Debouncer m_currentDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    
    private ShooterStates currentState;

    public double inputRPS;
    public double inputAngle;
    public boolean feederOn;

    public shooterSubsystem(){
        inputRPS = 0;
        inputAngle = 0;
        feederOn = false;

        currentState = ShooterStates.TEST;
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

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 35.0 / 360.0; // 45 degrees
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // 0 degrees
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;


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

    
        //Input Values
        SmartDashboard.putNumber("Shooter Inputs/Input Shooter RPS", inputRPS); 
        SmartDashboard.putNumber("Shooter Inputs/Input Hood Angle", inputAngle);
        SmartDashboard.putBoolean("Shooter Inputs/Enable Feeder", feederOn);
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

    public void driveHoodVoltage(double voltage){
        hoodMotor.setVoltage(voltage);
    }

    public void setSoftLimits(boolean active){
        var limitConfigs = new SoftwareLimitSwitchConfigs();
        limitConfigs.ForwardSoftLimitEnable = active;
        limitConfigs.ReverseSoftLimitEnable = false;
        hoodMotor.getConfigurator().apply(limitConfigs);     
        hoodMotor.getConfigurator().apply(limitConfigs);     
    }

    public void resetHoodEncoder(){
            hoodMotor.setPosition(0);
        }

    public void stopShooterMotors(){
        shooterLeaderMotor.stopMotor();
    }

    public void stopFeeder(){
        feederMotor.stopMotor();
    }

    @Override
    public void periodic(){
        //Current Values
        SmartDashboard.putNumber("Shooter Current/Shooter RPS", shooterLeaderMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Current/Hood Angle (degrees)", hoodMotor.getPosition().getValueAsDouble()*360);
        SmartDashboard.putString("Shooter Current/Shooter State", this.currentState.toString());

        if ((m_currentDebouncer.calculate(hoodMotor.getStatorCurrent().getValueAsDouble() > 20)&& hoodMotor.getVelocity().getValueAsDouble() < 1)&& currentState == ShooterStates.REZERO){
                driveHoodVoltage(0);
                resetHoodEncoder();
                setSoftLimits(false);
                this.currentState = ShooterStates.IDLE;
            }
        

    }


    public Command setState(SuperStructure.ShooterStates state){
        this.currentState = state;
        Command command;
        switch (state) {
            case AIMING:
            command = run(()->{
                var params = shotCalculator.getInstance().getParameters();
                setHoodAngle(params.hoodAngle());
                setShooterRPS(params.flywheelSpeed());
                stopFeeder();
            });
            break;

            case SHOOTING:
                command = run(()->{
                    var params = shotCalculator.getInstance().getParameters();
                    setHoodAngle(params.hoodAngle());
                    setShooterRPS(params.flywheelSpeed());
                    setFeederVoltage(7);
                });
            break;

            case TEST:
                command = run(()->{
                    inputRPS = SmartDashboard.getNumber("Shooter Inputs/Input Shooter RPS", inputRPS);
                    inputAngle = SmartDashboard.getNumber("Shooter Inputs/Input Hood Angle", inputAngle);
                    feederOn = SmartDashboard.getBoolean("Shooter Inputs/Enable Feeder", feederOn);
                    
                    setShooterRPS(inputRPS);
                    setFeederVoltage(feederOn ? 10: 0);
                    
                    setHoodAngle(inputAngle);
                    // setShooterRPS(inputRPS);
                    
                });
                break;

            case JAM:
                command = run(()->{
                    var params = shotCalculator.getInstance().getParameters();
                    setHoodAngle(params.hoodAngle());
                    setShooterRPS(params.flywheelSpeed());
                    setFeederVoltage(-7);

                });
        
            case IDLE:
                command = run(()->{
                    setHoodAngle(20);
                    stopFeeder();
                    stopShooterMotors();
                });
                break;

            case REZERO:
                command = run(()->{
                    currentState = ShooterStates.REZERO;
                    m_currentDebouncer.calculate(false);
                    driveHoodVoltage(-2);
                    setSoftLimits(false);
                    
                });
                break;

            default:
            command = run(()->{
                setHoodAngle(0);
                stopShooterMotors();
                stopFeeder();
            });
                break;
        };
        return command;
    }
}
