package frc.robot.subsystems;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
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
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;


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
    private double pivotSetpointDegrees;
    private double rollerSetpointRPM;
    private double simulatedPivotDegrees;
    private double simulatedPivotVelocityRps;
    private double simulatedRollerRPM;
    private final SparkMaxSim rollerMotorSim;




    public intakeSubsystem(){
        currentIntakePivotState = IntakePivotStates.PIVOT_START_POS;
        currentIntakeRollerState = IntakeRollerStates.ROLLER_OFF;
        inputPivotAngle = 0;
        inputRollerRPM = 0;
        pivotSetpointDegrees = IntakeConstants.startingPosAngle;
        rollerSetpointRPM = 0;
        simulatedPivotDegrees = IntakeConstants.startingPosAngle;

        rollerMotor = new SparkMax(IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pivotMotor = new TalonFX(IntakeConstants.pivotMotorID);
        pivotEncoder = new CANcoder(IntakeConstants.CANcoderID);
        rollerMotorSim = new SparkMaxSim(rollerMotor, DCMotor.getNeoVortex(1));
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
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
        pivotConfig.CurrentLimits.StatorCurrentLimit  = 120;

        pivotConfig.Feedback.RotorToSensorRatio = (4.0*5.0*(42.0/36.0));
        pivotConfig.Feedback.SensorToMechanismRatio = (32.0/14.0); 
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 3; 
        pivotConfig.MotionMagic.MotionMagicAcceleration = 1.5; // 2.3 before


        pivotConfig.Slot0.kS = 0.25;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kV = (0.12*(4.0*5.0*(42.0/36.0)*(32.0/14.0)));
        pivotConfig.Slot0.kP = 60;
        // pivotConfig.Slot0.kG = 0.35; 
        //  pivotConfig.Slot0.kD = 0;

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
                        .kV(IntakeConstants.rollerKvVoltsPerOutputRPM);
            rollerConfig.encoder.velocityConversionFactor(1.0 / IntakeConstants.rollerGearReduction);

        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //INPUT VALUES
        SmartDashboard.putNumber("Intake Inputs/Input Pivot Degrees", inputPivotAngle);
        SmartDashboard.putNumber("Intake Inputs/Input Roller RPM", inputRollerRPM);
        SmartDashboard.setDefaultNumber("Intake/RollerActiveTargetRPM", IntakeConstants.rollerRPM);

    }

    @Override
    public void periodic(){
        if (currentIntakeRollerState == IntakeRollerStates.ROLLER_ACTIVE) {
            setRollerRPM(SmartDashboard.getNumber(
                "Intake/RollerActiveTargetRPM", IntakeConstants.rollerRPM));
        }
        SmartDashboard.putString("Intake/RequestedPivotState", currentIntakePivotState.toString());
        SmartDashboard.putNumber("Intake/PivotSetpointDeg", pivotSetpointDegrees);
        SmartDashboard.putNumber("Intake/PivotActualDeg", getIntakePivotAngle());
        SmartDashboard.putBoolean("Intake/PivotAtSetpoint", pivotAtSetpoint(3.0));
        SmartDashboard.putNumber("Intake/RollerSetpointRPM", rollerSetpointRPM);
        SmartDashboard.putNumber("Intake/RollerActualRPM", getRollerRPM());
        SmartDashboard.putNumber("Intake/RollerAppliedOutput", rollerMotor.getAppliedOutput());
    }

    public void setPivotPosition(double angleDegrees){
        pivotSetpointDegrees = angleDegrees;
        angleDegrees = Units.degreesToRotations(angleDegrees);
        pivotMotor.setControl(m_MotionMagicVoltage.withPosition(angleDegrees));
    }

    public IntakePivotStates getState(){
        return this.currentIntakePivotState;
    }

    public IntakeRollerStates getRollerState() { return currentIntakeRollerState; }

    public void setRollerRPM(double RPM){
        rollerSetpointRPM = RPM;
        rollerMotor.getClosedLoopController().setSetpoint(RPM, ControlType.kVelocity);
    }

    public void stopRoller(){
        rollerSetpointRPM = 0;
        // rollerMotor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
        rollerMotor.stopMotor(); 
    }

    public boolean isDeployed(){
        return isDeployed;
    }

    public double getIntakePivotAngle(){
        if (RobotBase.isSimulation()) return simulatedPivotDegrees;
        return pivotMotor.getPosition().getValueAsDouble()*360; //degrees
    }

    public double getRollerRPM() {
        if (RobotBase.isSimulation()) return simulatedRollerRPM;
        return rollerMotor.getEncoder().getVelocity();
    }

    public double getRollerAppliedOutput() {
        return rollerMotor.getAppliedOutput();
    }

    public boolean pivotAtSetpoint(double toleranceDegrees) {
        return Math.abs(pivotSetpointDegrees - getIntakePivotAngle()) <= toleranceDegrees;
    }
    
    public Command toggleCommand() {
        return runOnce(() -> {
            if (isDeployed) {
                applyPivotState(IntakePivotStates.PIVOT_HOME);
                applyRollerState(IntakeRollerStates.ROLLER_OFF);
            } else {
                applyPivotState(IntakePivotStates.PIVOT_DEPLOYED);
                applyRollerState(IntakeRollerStates.ROLLER_ACTIVE);
            }
        });
    }

    public void applyRollerState(IntakeRollerStates state) {
        currentIntakeRollerState = state;
        switch (state) {
            case ROLLER_ACTIVE -> setRollerRPM(
                SmartDashboard.getNumber("Intake/RollerActiveTargetRPM", IntakeConstants.rollerRPM));
            case ROLLER_SLOW -> setRollerRPM(IntakeConstants.rollerSlow);
            case ROLLER_OUTTAKE -> setRollerRPM(IntakeConstants.rollerOutake);
            case ROLLER_TEST -> setRollerRPM(
                SmartDashboard.getNumber("Intake Inputs/Input Roller RPM", inputRollerRPM));
            default -> stopRoller();
        }
    }

    public void applyPivotState(IntakePivotStates state) {
        currentIntakePivotState = state;
        switch (state) {
            case PIVOT_HOME -> { setPivotPosition(IntakeConstants.homeAngle); isDeployed = false; }
            case PIVOT_TRAVEL -> { setPivotPosition(IntakeConstants.homeAngle); isDeployed = true; }
            case PIVOT_DEPLOYED -> { setPivotPosition(IntakeConstants.deployedAngle); isDeployed = true; }
            case PIVOT_TEST -> setPivotPosition(
                SmartDashboard.getNumber("Intake Inputs/Input Pivot Degrees", getIntakePivotAngle()));
            default -> setPivotPosition(IntakeConstants.startingPosAngle);
        }
    }

    public Command setRollerState(SuperStructure.IntakeRollerStates state){
        return runOnce(() -> applyRollerState(state));
    }


    public Command setPivotState(SuperStructure.IntakePivotStates state){
        return runOnce(() -> applyPivotState(state));
    }

    @Override
    public void simulationPeriodic() {
        double error = pivotSetpointDegrees - simulatedPivotDegrees;
        double step = Math.copySign(Math.min(Math.abs(error), 120.0 * 0.02), error);
        simulatedPivotDegrees += step;
        simulatedPivotVelocityRps = Units.degreesToRotations(step / 0.02);
        double sensorRotations = Units.degreesToRotations(simulatedPivotDegrees) * (32.0 / 14.0);
        pivotEncoder.getSimState().setRawPosition(sensorRotations);
        pivotEncoder.getSimState().setVelocity(simulatedPivotVelocityRps * (32.0 / 14.0));
        pivotMotor.getSimState().setRawRotorPosition(sensorRotations * (4.0 * 5.0 * (42.0 / 36.0)));
        pivotMotor.getSimState().setRotorVelocity(
            simulatedPivotVelocityRps * (32.0 / 14.0) * (4.0 * 5.0 * (42.0 / 36.0)));
        rollerMotorSim.iterate(
            rollerSetpointRPM * IntakeConstants.rollerGearReduction, 12.0, 0.02);
        simulatedRollerRPM = rollerSetpointRPM;
    }
    

}
