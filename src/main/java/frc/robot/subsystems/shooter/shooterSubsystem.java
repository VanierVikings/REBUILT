package frc.robot.subsystems.shooter;


import java.lang.reflect.Parameter;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;



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
    private double flywheelSetpointRPS;
    private double hoodSetpointDegrees;
    private double feederVoltage;
    private double simulatedFlywheelRPS;
    private double simulatedHoodDegrees;
    private final SparkMaxSim feederMotorSim;
    private static final double HOOD_SENSOR_TO_MECHANISM = 4.0 * (189.0 / 8.0);

    public shooterSubsystem(){
        inputRPS = 0;
        inputAngle = 0;
        feederOn = false;

        currentState = ShooterStates.HOME;
        shooterLeaderMotor = new TalonFX(ShooterConstants.shooterLeaderMotor);
        shooterFollowerMotor = new TalonFX(ShooterConstants.shooterFollowerMotor);
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorID);
        feederMotor = new SparkMax(ShooterConstants.feederMotorID, MotorType.kBrushless);
        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));

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
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        hoodConfig.MotionMagic.MotionMagicAcceleration = 2.5; //3.0

        hoodConfig.Feedback.SensorToMechanismRatio = (4.0*(189.0/8.0)); // 4:1 maxplanetary + 365:30 rack and pinion
        hoodConfig.Slot0.kS = 0.2;
        hoodConfig.Slot0.kV = (0.12*(4.0*(189.0/8.0)));
        hoodConfig.Slot0.kP = 200;
        hoodConfig.Slot0.kD = 0;

        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 60.0 / 360.0; // 45 degrees
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
        flywheelSetpointRPS = rps;
        shooterLeaderMotor.setControl(m_request.withVelocity(rps));
        //shooterFollowerMotor.setControl(m_request.withVelocity(rps));
 }

    public void setFeederVoltage(double voltage){
        feederVoltage = voltage;
        feederMotor.setVoltage(voltage);
    }

    public void setHoodAngle(double targetDegrees){
        hoodSetpointDegrees = targetDegrees;
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
        flywheelSetpointRPS = 0;
        shooterLeaderMotor.stopMotor();
    }

    public void startFeeder(){
        feederVoltage = 10;
        feederMotor.setVoltage(feederVoltage);
    }

    public void stopFeeder(){
        feederVoltage = 0;
        feederMotor.stopMotor();
    }

    public ShooterStates getState(){
        return this.currentState;
    }

    public boolean shooterAtSpeed(double tolerance){
        return Math.abs(flywheelSetpointRPS - getFlywheelRPS()) <= tolerance;
    }

    public boolean hoodAtAngle(double toleranceDegrees) {
        return Math.abs(hoodSetpointDegrees - getHoodDegrees()) <= toleranceDegrees;
    }

    public double getFlywheelRPS() {
        return RobotBase.isSimulation() ? simulatedFlywheelRPS
            : shooterLeaderMotor.getVelocity().getValueAsDouble();
    }

    public double getHoodDegrees() {
        return RobotBase.isSimulation() ? simulatedHoodDegrees
            : hoodMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public double getFeederVoltage() { return feederVoltage; }

    public void applyState(ShooterStates state, boolean feedEnabled) {
        currentState = state;
        switch (state) {
            case AIMING -> {
                stopFeeder();
                var params = shotCalculator.getInstance().getParameters();
                // setHoodAngle(45);
                // setShooterRPS(35);
                setHoodAngle(ShooterConstants.actualHoodAngle);
                setShooterRPS(ShooterConstants.rotationPerSecond);
                // setHoodAngle(params.hoodAngle());
                // setShooterRPS(params.flywheelSpeed());
                //setFeederVoltage(state == ShooterStates.SHOOTING && feedEnabled ? 7.0 : 0.0);
            }

            case SHOOTING -> {
                startFeeder();
                var params = shotCalculator.getInstance().getParameters();
                //setHoodAngle(25);
                //setShooterRPS(30);
                setHoodAngle(ShooterConstants.actualHoodAngle);
                setShooterRPS(ShooterConstants.rotationPerSecond);
                // setHoodAngle(params.hoodAngle());
                // setShooterRPS(params.flywheelSpeed());
                //setFeederVoltage(state == ShooterStates.SHOOTING && feedEnabled ? 7.0 : 0.0);
            }

            case JAM -> {
                var params = shotCalculator.getInstance().getParameters();
                setHoodAngle(params.hoodAngle());
                setShooterRPS(params.flywheelSpeed());
                setFeederVoltage(-7.0);
            }
            case TEST -> {
                inputRPS = SmartDashboard.getNumber("Shooter Inputs/Input Shooter RPS", inputRPS);
                inputAngle = SmartDashboard.getNumber("Shooter Inputs/Input Hood Angle", inputAngle);
                feederOn = SmartDashboard.getBoolean("Shooter Inputs/Enable Feeder", feederOn);
                setShooterRPS(inputRPS);
                setHoodAngle(inputAngle);
                setFeederVoltage(feederOn ? 10.0 : 0.0);
            }
            case REZERO -> {
                m_currentDebouncer.calculate(false);
                driveHoodVoltage(-2);
                setSoftLimits(false);
            }
            default -> stopAndHome();
        }
    }

    public void stopAndHome() {
        currentState = ShooterStates.IDLE;
        setHoodAngle(0);
        stopShooterMotors();
        stopFeeder();
    }

    @Override
    public void periodic(){
        //Current Values
        double actualRPS = getFlywheelRPS();
        SmartDashboard.putString("Shooter/RequestedState", currentState.toString());
        SmartDashboard.putNumber("Shooter/FlywheelSetpointRPS", flywheelSetpointRPS);
        SmartDashboard.putNumber("Shooter/FlywheelActualRPS", actualRPS);
        SmartDashboard.putNumber("Shooter/FlywheelErrorRPS", flywheelSetpointRPS - actualRPS);
        SmartDashboard.putBoolean("Shooter/FlywheelAtSpeed", shooterAtSpeed(2.0));
        SmartDashboard.putNumber("Shooter/HoodSetpointDeg", hoodSetpointDegrees);
        SmartDashboard.putNumber("Shooter/HoodActualDeg", getHoodDegrees());
        SmartDashboard.putBoolean("Shooter/HoodAtAngle", hoodAtAngle(2.0));
        SmartDashboard.putNumber("Shooter/FeederVoltage", feederVoltage);
        SmartDashboard.putBoolean("Shooter/HoodZeroAssumed", true);
        SmartDashboard.putNumber("hooddeg", ShooterConstants.actualHoodAngle);
        SmartDashboard.putNumber("rps", ShooterConstants.rotationPerSecond);

        if ((m_currentDebouncer.calculate(hoodMotor.getStatorCurrent().getValueAsDouble() > 20)&& hoodMotor.getVelocity().getValueAsDouble() < 1)&& currentState == ShooterStates.REZERO){
                driveHoodVoltage(0);
                resetHoodEncoder();
                setSoftLimits(false);
                this.currentState = ShooterStates.IDLE;
            }
        

    }

    //STEM Orientation op controls
    public Command opSlowShot(){
        return this.runEnd(() -> setShooterRPS(20), () -> stopShooterMotors());
    }

    public Command opFastShot(){
        return this.runEnd(()->setShooterRPS(40), ()->stopShooterMotors());
    }

    public Command opLowAngle(){
        return this.run(()-> setHoodAngle(30));
    }

    

    public Command runFeeder(){
        return this.runEnd(()->setFeederVoltage(10), ()->stopFeeder());
    }

    public Command shootHighAngle() {
    return this.runEnd(
        () -> {
            // Both periodic actions run together here
            setHoodAngle(shotCalculator.getInstance().getParameters().hoodAngle());
            setShooterRPS(shotCalculator.getInstance().getParameters().flywheelSpeed());
            setFeederVoltage(7);
        }, 
        () -> {
            // This runs when the trigger is released
            stopShooterMotors();
            stopFeeder();
            // If you need to stop the hood motor too, add it here
        }
    );
    }


    public Command setState(SuperStructure.ShooterStates state){
        return runEnd(() -> applyState(state, state == ShooterStates.SHOOTING), this::stopAndHome);
    }

    @Override
    public void simulationPeriodic() {
        simulatedFlywheelRPS += (flywheelSetpointRPS - simulatedFlywheelRPS) * 0.20;
        double hoodError = hoodSetpointDegrees - simulatedHoodDegrees;
        simulatedHoodDegrees += Math.copySign(Math.min(Math.abs(hoodError), 90.0 * 0.02), hoodError);
        shooterLeaderMotor.getSimState().setSupplyVoltage(12.0);
        shooterLeaderMotor.getSimState().setRotorVelocity(simulatedFlywheelRPS);
        hoodMotor.getSimState().setSupplyVoltage(12.0);
        hoodMotor.getSimState().setRawRotorPosition(
            Units.degreesToRotations(simulatedHoodDegrees) * HOOD_SENSOR_TO_MECHANISM);
        feederMotorSim.iterate(feederVoltage == 0 ? 0 : Math.copySign(3000, feederVoltage), 12.0, 0.02);
    }
}
