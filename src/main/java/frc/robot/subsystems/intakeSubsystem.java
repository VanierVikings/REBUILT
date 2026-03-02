package frc.robot.subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax; //doihfeweiufh
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;


public class intakeSubsystem extends SubsystemBase {
    private final SparkMax rollerMotor;
    private final TalonFX pivotMotor;
    private final SparkMaxConfig rollerConfig;
    private final TalonFXConfiguration pivotConfig;
    private final MotionMagicVoltage m_MotionMagicVoltage;
    private final DutyCycleEncoder pivotEncoder;
    private final double pivotEncoderZero;
    


    public intakeSubsystem(){
        pivotEncoderZero = 213.1;//aribitrary
        rollerMotor = new SparkMax(IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pivotMotor = new TalonFX(IntakeConstants.pivotMotorID);
        pivotEncoder = new DutyCycleEncoder(0); //DIO port, fullrange (test), expectedZero(Test)

        rollerConfig = new SparkMaxConfig();
        pivotConfig = new TalonFXConfiguration();

        m_MotionMagicVoltage = new MotionMagicVoltage(0);

        rollerConfig
            .smartCurrentLimit(40)
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;

        pivotConfig.Feedback.SensorToMechanismRatio = 69; //69 rotations of motor: 1 rotation of pivot
        // pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotConfig.Slot0.kS = 0;
        pivotConfig.Slot0.kV = 0;
        pivotConfig.Slot0.kP = 0;
        pivotConfig.Slot0.kI = 0;
        pivotConfig.Slot0.kP = 0;

        // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 90/360; //degrees TEST
        // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;



        pivotMotor.getConfigurator().apply(pivotConfig);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Pivot Encoder", pivotEncoder.get());
        SmartDashboard.putNumber("Motor encoder", pivotMotor.getPosition().getValueAsDouble());
    }

    public void runRoller(double direction){
        rollerMotor.set(direction);
    }
    public void stopRoller(){
        rollerMotor.stopMotor();
    }

    public void setIntakePosition(double targetDegrees){
        targetDegrees = Units.degreesToRotations(targetDegrees);
        pivotMotor.setControl(m_MotionMagicVoltage.withPosition(targetDegrees));
    }

    public void initIntakeAngle(){
        pivotMotor.setPosition(pivotEncoder.get());
    }

    public Command setRollerState(SuperStructure.IntakeRollerStates state){
        Command command;
        switch (state) {
            case ACTIVE:
                command = runOnce(()->{
                    runRoller(0.5);
                });
                break;

            case OFF:
            command = runOnce(()->{
                runRoller(0);
            });
            break;

            case SLOW:
            command = runOnce(()->{
                runRoller(0.1);
            });
            break;
        
            default: //stop
            command = runOnce(()->{
                runRoller(0);
            });
                break;
        }

        return command;
    }


    // public Command setPivotState(SuperStructure.IntakePivotStates states){
    //     Command command;

    //     switch (states) {
    //         case START_POS:
    //             setIntakePosition(pivotEncoderZero*360);
    //             break;
        
    //         default:
    //             break;
    //     }
    // }
    

}

