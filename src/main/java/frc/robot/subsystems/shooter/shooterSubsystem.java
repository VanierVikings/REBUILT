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
import com.revrobotics.spark.FeedbackSensor;
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
    private final TalonFX shooterRightMotor; //leaer
    private final TalonFX shooterLeftMotor; //follower
    
    private final SparkMax feederMotor;

    private final TalonFX hoodMotor;

    private final SparkMaxConfig feederConfig;


    
    
    private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0);
    private final MotionMagicVoltage m_MotionMagic = new MotionMagicVoltage(0);

    private ShooterStates currentState;

    private double inputRPS;
    private double inputHoodAngle;
    private boolean enableFeeder;

    private double targetRPS;



    public shooterSubsystem(){
        currentState = ShooterStates.HOME;
        inputRPS = 0;
        inputHoodAngle = 2;
        enableFeeder = false;

        shooterRightMotor = new TalonFX(ShooterConstants.shooterRightMotorID, "rio");
        shooterLeftMotor = new TalonFX(ShooterConstants.shooterLeftMotorID, "rio");
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorID);
        feederMotor = new SparkMax(ShooterConstants.feederMotorID, MotorType.kBrushless);

        feederConfig = new SparkMaxConfig();

        /*  Flywheel configs  */
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = 2;
        flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        // flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // flywheelConfig.CurrentLimits.StatorCurrentLimit = 60;

        flywheelConfig.Slot0.kS = 0.26;
        flywheelConfig.Slot0.kV = 0.12;
        flywheelConfig.Slot0.kP = 0.5;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterRightMotor.getConfigurator().apply(flywheelConfig);
        shooterLeftMotor.setControl(new Follower(ShooterConstants.shooterRightMotorID, MotorAlignmentValue.Opposed));

        shooterRightMotor.getVelocity().setUpdateFrequency(50);
        shooterLeftMotor.getVelocity().setUpdateFrequency(50);


        

        /*  Hood Configs  */
        var hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        // hoodMotorConfig.MotionMagic.MotionMagicAcceleration = 3;
        
        hoodMotorConfig.Feedback.SensorToMechanismRatio = (365/30);


        hoodMotorConfig.Slot0.kS = 0;//test
        // hoodMotorConfig.Slot0.kV = (0.12*(365/30));
        hoodMotorConfig.Slot0.kP = 0; //test
        hoodMotorConfig.Slot0.kI = 0; //test
        hoodMotorConfig.Slot0.kD = 0;//test

        hoodMotor.getConfigurator().apply(hoodMotorConfig);
        hoodMotor.setPosition(0);
        


        /* Feeder Configs */
        feederConfig
            .smartCurrentLimit(40)
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        }

        public void setShooterSpeed(double rps) {
            shooterRightMotor.setControl(m_VelocityVoltage.withVelocity(rps));
        }


        public void hoodVoltage(double volts) {
            hoodMotor.setVoltage(volts);
        }

        public void resetHoodEncoder(){
            hoodMotor.setPosition(0);
        }

        public void setHoodPosition(double degrees) {
            degrees = Units.degreesToRotations(degrees);
            hoodMotor.setControl(m_MotionMagic.withPosition(degrees));
        }

        public void stopShooter() {
            shooterRightMotor.stopMotor();
            shooterLeftMotor.stopMotor();
        }

        public void setFeederVoltage(double volts) {
            feederMotor.setVoltage(volts);
        }

        public void stopFeeder() {
            feederMotor.stopMotor();
        }

        public void setSoftLimits(boolean active) {
            var limitConfigs = new SoftwareLimitSwitchConfigs();
            limitConfigs.ForwardSoftLimitEnable = active;
            limitConfigs.ReverseSoftLimitEnable = false;
            hoodMotor.getConfigurator().apply(limitConfigs);
        }



        @Override
        public void periodic(){
            SmartDashboard.putNumber("Right RPS ", shooterRightMotor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Left RPS", shooterLeftMotor.getVelocity().getValueAsDouble());
            SmartDashboard.putString("Current Shooter State", currentState.toString());

            SmartDashboard.putNumber("Current Hood Degrees", hoodMotor.getPosition().getValueAsDouble()*360);
            SmartDashboard.putBoolean("Feeder On?", false);

           
        }


        public Command setState(SuperStructure.ShooterStates state){
            this.currentState = state;
            Command command;
            switch (state) {
                case TEST:
                       command = run(() -> {

                        double inputRPS = SmartDashboard.getNumber("Right RPS",0 );
                        double targetAngle = SmartDashboard.getNumber("Current Hood Angle", 0);
                        boolean shouldFeed = SmartDashboard.getBoolean("Feeder On?", false);

                        setHoodPosition(targetAngle);
                        setShooterSpeed(20); //please god work
                        setFeederVoltage(3);

                        System.out.println("green fn");
                        // if (shouldFeed) {
                        //     setFeederVoltage(10);
                        // } else {
                        //     stopFeeder();
                        // }
                    });
                    break;
            
                case IDLE:
                    command = run(()->{
                        setHoodPosition(0);
                        stopFeeder();
                        setShooterSpeed(0);

                    });
                    break;

                // case SHOOTING:
                //     command = run(()->{});
                //     break;

                // case REZERO:
                //     command = run(()->{
                //         currentState = ShooterStates.REZERO;
                //         resetHoodEncoder();
                //         // driveHoodVoltage(-2);
                //         // setSoftLimits(false);
                //     });
                //     break;

                default: //home
                command = run(()->{
                    setHoodPosition(0);
                    stopFeeder();
                    stopShooter();
                });
                break;
            };
            return command;
        }
}
