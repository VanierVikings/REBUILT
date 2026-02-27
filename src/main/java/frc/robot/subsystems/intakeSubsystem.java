// package frc.robot.subsystems;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.IntakeConstants;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.SparkMax; //doihfeweiufh
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.LimitSwitchConfig.Type;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkRelativeEncoder;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.MotorOutputStatusValue;


// public class intakeSubsystem extends SubsystemBase {
//     private final RelativeEncoder wheelEncoder;
//     private final SparkMax intakeWheelMotor;
//     private final TalonFX intakePivotMotor;
//     private final SparkMaxConfig wheelConfig;
//     // private final SparkLimitSwitch wheelLimitSwitch;
//     private final MotionMagicExpoVoltage motionMagic;
    
//     public enum setPoint {
//         rest,
//         down
//     }
 
//     @Override
//     public void periodic(){

//     }


//     public intakeSubsystem(){
//         zeroPivot();
//         intakeWheelMotor = new SparkMax(IntakeConstants.intakeWheelMotor, MotorType.kBrushless);
//         wheelEncoder = intakeWheelMotor.getEncoder();
//         wheelConfig = new SparkMaxConfig();
//         wheelConfig
//         .smartCurrentLimit(IntakeConstants.smartCurrentLimit)
//         .idleMode(IdleMode.kBrake);

// //         wheelLimitSwitch = intakeWheelMotor.getForwardLimitSwitch();

// //         wheelConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

//         intakeWheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//         intakePivotMotor = new TalonFX(IntakeConstants.intakePivotMotor);

//         TalonFXConfiguration config = new TalonFXConfiguration();

//         // PID
//         config.Slot0.kP = 80;
//         config.Slot0.kI = 0;
//         config.Slot0.kD = 1;

//         // Feedforward
//         config.Slot0.kS = 0.3;   // static friction
//         config.Slot0.kV = 0.12;  // velocity FF
//         config.Slot0.kG = 0.6;   // gravity FF (IMPORTANT for slapdown)

//         // Motion Magic settings
//         config.MotionMagic.MotionMagicCruiseVelocity = 100; // rotations/sec
//         config.MotionMagic.MotionMagicAcceleration = 200;
//         config.MotionMagic.MotionMagicExpo_kA = 0.2;
//         config.MotionMagic.MotionMagicJerk = 1000;
//         config.Feedback.SensorToMechanismRatio = 0; //gear ratio

//         CurrentLimitsConfigs current = new CurrentLimitsConfigs();
//         current.SupplyCurrentLimit = 40;
//         current.SupplyCurrentLimitEnable = true;
//         config.CurrentLimits = current;

//         intakePivotMotor.getConfigurator().apply(config);
//         motionMagic = new MotionMagicExpoVoltage(0);
//     }
    

//     public void zeroPivot(){
//         intakePivotMotor.setPosition(0.0);
//     }

//     public void setPivotPosition(double rotations) {
//         intakePivotMotor.setControl(motionMagic.withPosition(rotations));
//     }


//     public Command setPivot(setPoint setpoint) {
//         return this.runOnce(
//         () -> {
//           switch (setpoint) {
//             case rest:
//                 //
//                 break;
//             case down:
//                 //
//                 break;
//           }
//         });
//     }
// }

