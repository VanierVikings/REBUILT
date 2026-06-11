package frc.robot.subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SuperStructure.IntakePivotStates;
import frc.robot.subsystems.SuperStructure.IntakeRollerStates;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax; //doihfeweiufh
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


public class intakeSubsystem extends SubsystemBase {
    private final SparkMax rollerMotor;
    private final TalonFX pivotMotor;
    private final CANcoder pivotEncoder;
    private final MotionMagicVoltage m_MotionMagicVoltage;
    
    private SuperStructure.IntakePivotStates currentIntakePivotState;
    private SuperStructure.IntakeRollerStates currentIntakeRollerState;
    private boolean isDeployed;
    private double inputPivotAngle;
    private double inputRollerRPM;




    public intakeSubsystem(){
        currentIntakePivotState = IntakePivotStates.PIVOT_START_POS;
        currentIntakeRollerState = IntakeRollerStates.ROLLER_OFF;
        inputPivotAngle = 0;
        inputRollerRPM = 0;

        rollerMotor = new SparkMax(IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pivotMotor = new TalonFX(IntakeConstants.pivotMotorID);
        pivotEncoder = new CANcoder(IntakeConstants.CANcoderID);
        m_MotionMagicVoltage = new MotionMagicVoltage(0);


        /*  --- WCP THROUGBORE ENCODER VIA CANCODER --- */   
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; //subject to change
        encoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.CANcoderOffset;
        pivotEncoder.getConfigurator().apply(encoderConfig);



        /* --- INTAKE PIVOT VIA KRAKEN X60 */
        var pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 80;
        pivotConfig.CurrentLimits.StatorCurrentLimit  = 120;

        pivotConfig.Feedback.RotorToSensorRatio = (4.0*5.0*(42.0/36.0));
        pivotConfig.Feedback.SensorToMechanismRatio = (32.0/14.0); 
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 2; 
        pivotConfig.MotionMagic.MotionMagicAcceleration = 3.5;


        pivotConfig.Slot0.kS = 0.25;
        pivotConfig.Slot0.kV = (0.12*(4.0*5.0*(42.0/36.0)*(32.0/14.0)));
        pivotConfig.Slot0.kP = 0;
        pivotConfig.Slot0.kG = 0; 
         pivotConfig.Slot0.kD = 0;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90/360; //degrees TEST
        // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;


        pivotMotor.getConfigurator().apply(pivotConfig);
        isDeployed = false;



        /* --- INTAKE ROLLER CONFIG VIA NEO VORTEX --- */
        var rollerConfig = new SparkMaxConfig();
        rollerConfig
            .smartCurrentLimit(40)
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .closedLoop
                    .p(0)
                    .feedForward
                        .kS(0) //TEST
                        .kV((1.0/565.0));
        
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //INPUT VALUES
        SmartDashboard.putNumber("Intake Inputs/Input Pivot Degrees", inputPivotAngle);
        SmartDashboard.putNumber("Intake Inputs/Input Roller RPM", inputRollerRPM);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake/Intake CANcoder Angle", pivotEncoder.getPosition().getValueAsDouble()*360);
        SmartDashboard.putNumber("Intake/Intake Current Intake Angle", pivotMotor.getPosition().getValueAsDouble()*360);
        SmartDashboard.putString("Intake/Current Pivot State", currentIntakePivotState.toString());
        SmartDashboard.putString("Intake/Current Roller State", currentIntakeRollerState.toString());
    }

    public void setPivotPosition(double angleDegrees){
        angleDegrees = Units.degreesToRotations(angleDegrees);
        pivotMotor.setControl(m_MotionMagicVoltage.withPosition(angleDegrees));
    }

    public IntakePivotStates getState(){
        return this.currentIntakePivotState;
    }

    public void setRollerRPM(double RPM){
        rollerMotor.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
    }

    public void stopRoller(){
        // rollerMotor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
        rollerMotor.stopMotor(); 
    }

    public boolean isDeployed(){
        return isDeployed;
    }

    public double getIntakePivotAngle(){
        return pivotMotor.getPosition().getValueAsDouble()*360; //degrees
    }
    
    public Command opRoller(){
        return this.runEnd(() -> setRollerRPM(6000), () -> stopRoller());
    }


    public Command setRollerState(SuperStructure.IntakeRollerStates state){
        this.currentIntakeRollerState = state;
        Command command;
        switch (state) {
            case ROLLER_ACTIVE:
                command = runOnce(()->{
                    setRollerRPM(IntakeConstants.rollerRPM);
                });
                break;

            case ROLLER_OFF:
            command = runOnce(()->{
                stopRoller();
            });
            break;

            case ROLLER_SLOW:
            command = runOnce(()->{
                setRollerRPM(IntakeConstants.rollerSlow);
            });
            break;

            case ROLLER_OUTTAKE:
            command = runOnce(()->{
                setRollerRPM(IntakeConstants.rollerOutake);
            });
            break;
        
            case ROLLER_TEST:
            command = runOnce(()->{
                inputRollerRPM = SmartDashboard.getNumber("Intake Inputs/Input Roller RPM", inputRollerRPM);
                setRollerRPM(inputRollerRPM);
            });
            break;

            default: //stop
            command = runOnce(()->{
                setRollerRPM(0);
            });
                break;
        }

        return command;
    }


    public Command setPivotState(SuperStructure.IntakePivotStates state){
        this.currentIntakePivotState = state;
        Command command;
        switch (state) {
            case PIVOT_HOME:
                command = runOnce(()->{
                    setPivotPosition(IntakeConstants.homeAngle);
                    stopRoller();
                    isDeployed = false;
                });
                break;

            case PIVOT_TRAVEL:
            command = runOnce(()->{
                setPivotPosition(IntakeConstants.homeAngle);
                stopRoller();
                isDeployed = true;
            });
            break;

            case PIVOT_DEPLOYED:
            command = runOnce(()->{
                setPivotPosition(IntakeConstants.deployedAngle);
                stopRoller();
                isDeployed = true;
            });
            break;

            case PIVOT_TEST:
            command = runOnce(()->{
                // setPivotPosition(testAngle);
                inputPivotAngle = SmartDashboard.getNumber("Intake Inputs/Input Pivot Degrees", pivotMotor.getPosition().getValueAsDouble()*360);
                setPivotPosition(inputPivotAngle);
            });
            break;

            default: //stop
            command = runOnce(()->{
                setPivotPosition(IntakeConstants.startingPosAngle);
                stopRoller();
            });
                break;
        }

        return command;
    }
    

}