package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooterSubsystem;
import frc.robot.subsystems.shooter.shotCalculator;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class SuperStructure extends SubsystemBase{

    public enum ShooterStates{
        HOME, AIMING, SHOOTING, TEST, IDLE, REZERO
    }

    public enum CLimberStates{
        HOME, EXTENDED, RETRACTED, REZERO, TEST
    }

    public enum IntakePivotStates{
        START_POS, HOME, DEPLOYED, TRAVEL, AGITATING
    }

    public enum IntakeRollerStates{
        ACTIVE, OFF, SLOW
    }

    public enum SpindexerStates{
        FEED, OFF, SLOW, JAM
    }


    private shooterSubsystem m_shooter;
    private intakeSubsystem m_intake;
    private climbSubsystem m_climber;
    private SwerveSubsystem m_drive;

    private ShooterStates shooterStates = ShooterStates.HOME;
    private IntakePivotStates intakeStates = IntakePivotStates.START_POS;
    public boolean intaking;

    public SuperStructure(shooterSubsystem shooter, intakeSubsystem intake, climbSubsystem climber){
        this.m_shooter = shooter;
        this.m_intake = intake;
        this.m_climber = climber;
        this.intaking = false;
        shotCalculator.getInstance(m_drive);
    }

    private void setShooterState(ShooterStates requestedState){
        boolean isActionState = (requestedState == ShooterStates.AIMING || 
                                requestedState == ShooterStates.SHOOTING || 
                                requestedState == ShooterStates.TEST);
        // boolean isIntakeSafe = (m_intake.getIntakeAngle() < 120);

        ShooterStates finalState = (isActionState /*&& !isIntakeSafe*/) 
                                        ? this.shooterStates 
                                        : requestedState;

         if (requestedState == ShooterStates.TEST){
            this.shooterStates = ShooterStates.TEST;
        }
        Command stateCommand = m_shooter.setState(finalState);

        CommandScheduler.getInstance().schedule(stateCommand);;
    
    }
}

