package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ShooterStates;
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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.MathUtil;



public class shooterSubsystem extends SubsystemBase {
    private final SparkMax shooterLeftMotor;
    private final SparkMax shooterRightMotor;
    private final SparkMax feederMotor;
    private final TalonFX hoodMotor;
    private final SparkMaxConfig rightFlyWheelConfig;
    private final SparkMaxConfig leftFlywheelConfig;

    private final TalonFXConfiguration hoodMotorConfig;

    
    private final SparkClosedLoopController rightFlyWheelController;
    private final SparkClosedLoopController feederController;

    private final MotionMagicVoltage m_MotionMagic = new MotionMagicVoltage(0);

    private ShooterStates currentState;


    public shooterSubsystem(){
        currentState = ShooterStates.HOME;

        shooterRightMotor = new SparkMax(ShooterConstants.shooterCenterMotorID, MotorType.kBrushless);
        shooterLeftMotor = new SparkMax(ShooterConstants.shooterTopSpinMotorID,MotorType.kBrushless);
        feederMotor = new SparkMax(ShooterConstants.feederMotorID, MotorType.kBrushless);
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorID);

        rightFlyWheelConfig = new SparkMaxConfig();
        leftFlywheelConfig = new SparkMaxConfig();

        hoodMotorConfig = new TalonFXConfiguration();


        rightFlyWheelController = shooterRightMotor.getClosedLoopController();
        feederController = feederMotor.getClosedLoopController();


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

        // public boolean shooterAtSpeed(double velocityTolerance){
        //     return(shooter)
        // }
    
        public void runFeeder(double direction){
            feederMotor.set(direction);
        }

        public void stopFeeder(){
            feederMotor.set(0);
        }

        public void setFeederVelocity(double rps){
            feederController.setSetpoint((rps*60), ControlType.kMAXMotionVelocityControl); // maxmotion takes rpm ughh
        }

        public void setShooterSpeed(double rps){
            rightFlyWheelController.setSetpoint(rps, ControlType.kVelocity);
        }

        public void stopShooter(){
            rightFlyWheelController.setSetpoint(0, ControlType.kVelocity);
        }

        public void driveHoodVoltage(double voltage){
            hoodMotor.setVoltage(voltage);
        }

        public void setHoodPosition(double targetDegrees){
            targetDegrees = Units.degreesToRotations(targetDegrees);
            hoodMotor.setControl(m_MotionMagic.withPosition(targetDegrees));
        }

        public void resetHoodEncoder(){
            hoodMotor.setPosition(0);
        }

        public void setSoftLimits(boolean active){
            var limitConfigs = new SoftwareLimitSwitchConfigs();
            limitConfigs.ForwardSoftLimitEnable = active;
            limitConfigs.ReverseSoftLimitEnable = false;
            hoodMotor.getConfigurator().apply(limitConfigs);     
            hoodMotor.getConfigurator().apply(limitConfigs);     
        }



        @Override
        public void periodic(){
            SmartDashboard.putNumber("Shooter RPS", shooterRightMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Hood Angle degrees", hoodMotor.getPosition().getValueAsDouble());
            SmartDashboard.putString("Current Shooter State", currentState.toString());

        }


        public Command setState(SuperStructure.ShooterStates state){
            this.currentState = state;
            Command command;
            switch (state) {
                case TEST:
                       command = run(() -> {

                        double targetRPS = SmartDashboard.getNumber("Input RPS",0 );
                        double targetAngle = SmartDashboard.getNumber("Input Hood Angle", 0);
                        boolean shouldFeed = SmartDashboard.getBoolean("Enable Feeder", false);

                        // Apply to hardware
                        setHoodPosition(targetAngle);
                        setShooterSpeed(targetRPS);
                        setFeederVelocity(shouldFeed ? 75 : 0);
                    });
                    break;
            
                // case IDLE:
                //     command = run(()->{});
                //     break;

                // case SHOOTING:
                //     command = run(()->{});
                //     break;

                case REZERO:
                    command = run(()->{
                        currentState = ShooterStates.REZERO;
                        resetHoodEncoder();
                        // driveHoodVoltage(-2);
                        setSoftLimits(false);
                    });
                    break;

                default: //home
                command = run(()->{
                    setHoodPosition(0);
                });
                break;
            };
            return command;
        }
}
