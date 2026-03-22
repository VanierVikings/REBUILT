package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.DriveStates;
import frc.robot.subsystems.SuperStructure.ShooterStates;
import frc.robot.subsystems.shooter.shooterSubsystem;
import frc.robot.subsystems.shooter.shotCalculator;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class SuperStructure extends SubsystemBase{

    public enum LedStates {
        RED_GR, BLUE_GR, Off
    }

    public enum ShooterStates{
        HOME, AIMING, SHOOTING, TEST, IDLE, REZERO, JAM
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

    public enum DriveStates{
        FIELD, AIMING, SOFT
    }

    private shooterSubsystem m_shooter;
    private spindexerSubsystem m_spindexer;
    // private intakeSubsystem m_intake;
    // private climbSubsystem m_climber;
    private SwerveSubsystem m_drive;

    private ShooterStates shooterStates = ShooterStates.HOME;
    private SpindexerStates spindexerStates = SpindexerStates.OFF;
    private IntakePivotStates intakeStates = IntakePivotStates.START_POS;
    private DriveStates driveStates = DriveStates.FIELD;

    public boolean intaking;

    // public SuperStructure(shooterSubsystem shooter, intakeSubsystem intake, climbSubsystem climber){

    public SuperStructure(shooterSubsystem shooter, spindexerSubsystem spindexer, SwerveSubsystem drive){
        this.m_shooter = shooter;
        this.m_spindexer = spindexer;
        this.m_drive = drive;
        // this.m_intake = intake;
        // this.m_climber = climber;
        intaking = false;
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

        CommandScheduler.getInstance().schedule(stateCommand);
    
    }

    private void setSpindexerState(SpindexerStates requestedStates){
        this.spindexerStates = requestedStates;
        Command stateCommand = m_spindexer.setState(requestedStates);

        // stateCommand.schedule();
        CommandScheduler.getInstance().schedule(stateCommand);
    }


    public Command driveRequest(DriveStates requestedState) {
        return run(() -> {
            this.driveStates = requestedState;
        }).finallyDo((interrupted) -> {
            this.driveStates = DriveStates.FIELD;
        });
    }

    public Command shooterRequest(ShooterStates requestedState) {
        return run(() -> {
            setShooterState(requestedState);
        }).finallyDo((interrupted) -> {
            setShooterState(ShooterStates.IDLE);
        });
    }


    public Command hopperRequest(SpindexerStates requestedState) {
    return run(() -> setSpindexerState(requestedState))
           .finallyDo((interrupted) -> setSpindexerState(SpindexerStates.OFF));
    }


    public Command firingCommand(ShooterStates sState, SpindexerStates spinState, DriveStates dState) {
        return runOnce(() -> {
            setShooterState(ShooterStates.JAM);
            setSpindexerState(SpindexerStates.JAM);
            this.driveStates = dState;
        })
        .andThen(runOnce(()->{
            setShooterState(sState);
            setSpindexerState(spinState);
        }))
        .andThen(
            Commands.either(
                runOnce(() -> {
                        // setIntakeState(IntakeStates.AGITATING);
                        // setIntakeRollerState(IntakeRollerState.INTAKE_SLOW);
                    }),
                Commands.none(),
                () -> /*this.intakeState == IntakeStates.INTAKE_TRAVEL &&*/ sState == ShooterStates.SHOOTING
            )
        )
        .andThen(Commands.idle()) 
        .finallyDo((interrupted) -> {
            setShooterState(ShooterStates.IDLE);
            setSpindexerState(spindexerStates.OFF);
            this.driveStates = DriveStates.FIELD;
            // if (this.intakeState == IntakeStates.AGITATING){
            //     setIntakeState(IntakeStates.INTAKE_TRAVEL);
            //     setIntakeRollerState(IntakeRollerState.INTAKE_OFF);
            // }
        });
    }
}

