package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        PIVOT_START_POS, PIVOT_HOME, PIVOT_DEPLOYED, PIVOT_TRAVEL, PIVOT_AGITATING, PIVOT_TEST
    }

    public enum IntakeRollerStates{
        ROLLER_ACTIVE, ROLLER_OFF, ROLLER_SLOW,ROLLER_OUTTAKE, ROLLER_TEST
    }

    public enum SpindexerStates{
        FEED, OFF, SLOW, JAM
    }

    public enum DriveStates{
        FIELD, AIMING, SOFT
    }

    private shooterSubsystem m_shooter;
    private spindexerSubsystem m_spindexer;
    private intakeSubsystem m_intake;
    // private climbSubsystem m_climber;
    private SwerveSubsystem m_drive;

    private ShooterStates shooterState = ShooterStates.HOME;
    private SpindexerStates spindexerState = SpindexerStates.OFF;
    private IntakePivotStates intakeState = IntakePivotStates.PIVOT_START_POS;
    private DriveStates driveState = DriveStates.FIELD;

    public boolean intaking;

    // public SuperStructure(shooterSubsystem shooter, intakeSubsystem intake, climbSubsystem climber){

    public SuperStructure(shooterSubsystem shooter, spindexerSubsystem spindexer, intakeSubsystem intake, SwerveSubsystem drive){
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
                                        ? this.shooterState 
                                        : requestedState;

         if (requestedState == ShooterStates.TEST){
            this.shooterState = ShooterStates.TEST;
        }
        Command stateCommand = m_shooter.setState(finalState);

        CommandScheduler.getInstance().schedule(stateCommand);
    
    }

    private void setSpindexerState(SpindexerStates requestedStates){
        this.spindexerState = requestedStates;
        Command stateCommand = m_spindexer.setState(requestedStates);

        // stateCommand.schedule();
        CommandScheduler.getInstance().schedule(stateCommand);
    }

    private void setIntakePivotState(IntakePivotStates requestedState){
        this.intakeState = requestedState;
        Command stateCommand = m_intake.setPivotState(requestedState);
        CommandScheduler.getInstance().schedule(stateCommand);
    }

    private void setIntakeRollerState(IntakeRollerStates rollerState){
        this.intaking = (rollerState == IntakeRollerStates.ROLLER_ACTIVE);
        Command stateCommand = m_intake.setRollerState(rollerState);
        CommandScheduler.getInstance().schedule(stateCommand);
    }


    
    /* --- COMMANDS --- */

    public Command driveRequest(DriveStates requestedState) {
        return run(() -> {
            this.driveState = requestedState;
        }).finallyDo((interrupted) -> {
            this.driveState = DriveStates.FIELD;
        });
    }

    public Command shooterRequest(ShooterStates requestedState) {
        return run(() -> {
            setShooterState(requestedState);
        }).finallyDo((interrupted) -> {
            setShooterState(ShooterStates.IDLE);
        });
    }


    public Command firingCommand(ShooterStates sState, SpindexerStates spinState, DriveStates dState) {
        return runOnce(() -> {
            setShooterState(ShooterStates.JAM);
            setSpindexerState(SpindexerStates.JAM);
            this.driveState = dState;
        })
        .andThen(
            Commands.waitSeconds(0.1)
            .andThen(runOnce(()->{
                setShooterState(sState);
                setSpindexerState(spinState);
            }))
        )
        .andThen(
            Commands.either(
                // Path if condition is true: replaced intake agitation with none
                Commands.none(),
                /* Commands.waitSeconds(0.65)
                    .andThen(runOnce(() -> {
                        setIntakePivotState(IntakePivotStates.AGITATING);
                        setIntakeRollerState(IntakeRollerStates.ROLLER_SLOW);
                    })
                ), 
                */
                // Path if condition is false
                Commands.none(),
                // Condition to check
                () -> this.intakeState == IntakePivotStates.PIVOT_TRAVEL && sState == ShooterStates.SHOOTING
            )
        )
        .andThen(Commands.idle()) 
        .finallyDo((interrupted) -> {
            setShooterState(ShooterStates.IDLE);
            setSpindexerState(SpindexerStates.OFF);
            this.driveState = DriveStates.FIELD;
            
            // Commented out intake reset logic
            /* if (this.intakeState == IntakePivotStates.AGITATING){
                setIntakePivotState(IntakePivotStates.PIVOT_TRAVEL);
                setIntakeRollerState(IntakeRollerStates.ROLLER_OFF);
            } 
            */
        });
    }
}

