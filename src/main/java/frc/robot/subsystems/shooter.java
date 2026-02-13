package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.MathUtil;



public class shooter extends SubsystemBase {
    private final TalonFX shooterMotorTopMaster;
    private final TalonFX shooterMotorTopSlave;
    private final SparkMax hoodMotor;
    private final TalonFXConfiguration shooterConfig;
    private final VelocityVoltage m_request;
    private final InterpolatingDoubleTreeMap rpmMap;
    private final InterpolatingDoubleTreeMap angleMap;
    private final SparkMaxConfig hoodMotorConfig;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodClosedLoopController;
    
    public shooter(){
        shooterMotorTopMaster = new TalonFX(ShooterConstants.shooterMotorTopMasterID);
        shooterMotorTopSlave = new TalonFX(ShooterConstants.shooterMotorTopSlaveID);

        hoodMotor = new SparkMax(ShooterConstants.shooterMotorBottomID, SparkMax.MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        hoodMotorConfig = new SparkMaxConfig();
        hoodMotorConfig.closedLoop
                        .p(0)
                        .i(0)
                        .d(0)
                        .outputRange(0, 0);
        
        hoodMotorConfig.closedLoop.maxMotion
            .cruiseVelocity(0)
            .maxAcceleration(0)
            .allowedProfileError(0);

        hoodMotorConfig.smartCurrentLimit(Constants.ShooterConstants.HOOD_CURRENT_LIMIT).inverted(false).idleMode(IdleMode.kBrake);
        
        hoodMotor.configure(hoodMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        hoodClosedLoopController = hoodMotor.getClosedLoopController();

        shooterConfig = new TalonFXConfiguration();
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;

        rpmMap = new InterpolatingDoubleTreeMap();
        angleMap = new InterpolatingDoubleTreeMap();

        rpmMap.put(0.0, 0.0); //exmaples, we will add them later when testing
        angleMap.put(0.0, 0.0);

        shooterMotorTopSlave.setControl(new Follower(ShooterConstants.shooterMotorTopMasterID, MotorAlignmentValue.Opposed));

        Slot0Configs slot0Config = new Slot0Configs();
        slot0Config.kV = 0.1; // Example Feedforward gain (output per unit of requested velocity, in this case, output/rps)
        slot0Config.kP = Constants.ShooterConstants.kShooterP; // Example Proportional gain (output per unit of error in velocity)
        slot0Config.kI = Constants.ShooterConstants.kShooterI;
        slot0Config.kD = Constants.ShooterConstants.kShooterD; 
        shooterMotorTopMaster.getConfigurator().apply(slot0Config);

        m_request = new VelocityVoltage(0).withSlot(0);


    }

    public void setRPM(double rpm) {
    double rps = rpm / 60.0; // Phoenix uses rotations per second
    shooterMotorTopMaster.setControl(m_request.withVelocity(rps));
    }

    public void setHoodAngle(double degrees) {
        double rotations = degrees/Constants.ShooterConstants.degreesPerRotation;
        hoodClosedLoopController.setSetpoint(rotations, ControlType.kMAXMotionVelocityControl);
    }


    public void aimAndSpin(double distanceMeters) {
        distanceMeters = MathUtil.clamp(distanceMeters, 
                                        Constants.ShooterConstants.minDistance, 
                                        Constants.ShooterConstants.maxDistance); //make sure the hood doesnt kill itself 

        double rpm   = rpmMap.get(distanceMeters);
        double angle = angleMap.get(distanceMeters);

        setRPM(rpm);        // Kraken velocity PID
        setHoodAngle(angle); // NEO position PID
    }    
    
}
