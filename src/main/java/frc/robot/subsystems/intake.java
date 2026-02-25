// [package frc.robot.subsystems;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.IntakeConstants;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.SparkMaxPIDController;
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


// public class intake extends SubsystemBase {
//     private final RelativeEncoder wheelEncoder;
//     private final SparkMax intakeWheelMotor;
//     private final TalonFX intakePivotMotor;
//     private final SparkMaxConfig wheelConfig;
//     private final SparkLimitSwitch wheelLimitSwitch;

    
//     /*public enum setPoint {
//         rampUp,
//         rampDown
//     }
//     public intake() {
        
//     }
//     @Override
//     public void periodic(){

//     }*/


//     public intake(){
//         wheelEncoder = intakeWheelMotor.getEncoder();
//         intakeWheelMotor = new SparkMax(motorID, MotorType.kBrushless);
//         wheelConfig = new SparkMaxConfig();
//         wheelconfig
//         .smartCurrentLimit(constant)
//         .idleMode(constant);

//         wheelLimitSwitch = intakeWheelMotor.getForwardLimitSwitch();

//         wheelConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

//         intakeWheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }
    




//     public Command setPivot(Setpoint setpoint) {
//         return this.runOnce(
//         () -> {
//           switch (setpoint) {
//             case rampUp:
//                 //
//                 break;
//             case rampDown:
//                 //
//                 break;
//           }
//         });
//     }
// }
