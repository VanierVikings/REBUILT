package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooterSubsystem;
import frc.robot.subsystems.shooter.shotCalculator;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Coordinates mechanism requests without scheduling child commands from execute loops. */
public class SuperStructure extends SubsystemBase {
    public enum LedStates { RED_GR, BLUE_GR, Off }
    public enum ShooterStates { HOME, AIMING, SHOOTING, TEST, IDLE, REZERO, JAM }
    public enum CLimberStates { HOME, EXTENDED, RETRACTED, REZERO, TEST }
    public enum IntakePivotStates {
        PIVOT_START_POS, PIVOT_HOME, PIVOT_DEPLOYED, PIVOT_TRAVEL, PIVOT_AGITATING, PIVOT_TEST
    }
    public enum IntakeRollerStates {
        ROLLER_ACTIVE, ROLLER_OFF, ROLLER_SLOW, ROLLER_OUTTAKE, ROLLER_TEST
    }
    public enum SpindexerStates { FEED, OFF, SLOW, JAM }
    public enum DriveStates { FIELD, AIMING, SOFT }

    public static final double FLYWHEEL_TOLERANCE_RPS = 2.0;
    public static final double HOOD_TOLERANCE_DEGREES = 2.0;
    public static final double HEADING_TOLERANCE_DEGREES = 3.0;
    private static final double READY_DEBOUNCE_SECONDS = 0.100;

    private final shooterSubsystem shooter;
    private final spindexerSubsystem spindexer;
    private final intakeSubsystem intake;
    private final SwerveSubsystem drive;
    private final Debouncer readyDebouncer =
        new Debouncer(READY_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);

    private boolean aimRequested;
    private boolean shootRequested;
    private boolean readyToFeed;
    private boolean feedEnabled;

    public SuperStructure(
            shooterSubsystem shooter,
            spindexerSubsystem spindexer,
            intakeSubsystem intake,
            SwerveSubsystem drive) {
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.intake = intake;
        this.drive = drive;
        shotCalculator.getInstance(drive);
    }

    public Command toggleIntakeCommand() {
        return intake.toggleCommand();
    }

    public Command aimCommand(DoubleSupplier xInput, DoubleSupplier yInput) {
        return coordinatedShotCommand(false, xInput, yInput);
    }

    public Command shootCommand(DoubleSupplier xInput, DoubleSupplier yInput) {
        return coordinatedShotCommand(true, xInput, yInput);
    }

    private Command coordinatedShotCommand(
            boolean shooting, DoubleSupplier xInput, DoubleSupplier yInput) {
        Command readiness = Commands.run(() -> {
            boolean rawReady = shooter.shooterAtSpeed(FLYWHEEL_TOLERANCE_RPS)
                && shooter.hoodAtAngle(HOOD_TOLERANCE_DEGREES)
                && driveAtHeading();
            readyToFeed = readyDebouncer.calculate(rawReady);
            feedEnabled = shooting && readyToFeed;
        }, this);

        Command shooterCommand = shooter.runEnd(
            () -> shooter.applyState(
                shooting ? ShooterStates.SHOOTING : ShooterStates.AIMING,
                shooting && readyToFeed),
            shooter::stopAndHome);

        Command spindexerCommand = spindexer.runEnd(
            () -> spindexer.applyState(
                shooting && readyToFeed ? SpindexerStates.FEED : SpindexerStates.OFF),
            () -> spindexer.applyState(SpindexerStates.OFF));

        Command alignCommand = drive.SwerveControllerDrive(
            null, xInput, yInput, this::getTargetHeading, null, true);

        return Commands.parallel(readiness, shooterCommand, spindexerCommand, alignCommand)
            .beforeStarting(() -> {
                aimRequested = !shooting;
                shootRequested = shooting;
                readyToFeed = false;
                feedEnabled = false;
                readyDebouncer.calculate(false);
            })
            .finallyDo(interrupted -> {
                aimRequested = false;
                shootRequested = false;
                readyToFeed = false;
                feedEnabled = false;
                readyDebouncer.calculate(false);
                shooter.stopAndHome();
                spindexer.applyState(SpindexerStates.OFF);
            });
    }

    private Rotation2d getTargetHeading() {
        return Rotation2d.fromRadians(
            shotCalculator.getInstance().getParameters().robotHeadingRadians());
    }

    public boolean driveAtHeading() {
        return Math.abs(getHeadingErrorDegrees()) <= HEADING_TOLERANCE_DEGREES;
    }

    public double getHeadingErrorDegrees() {
        return getTargetHeading().minus(drive.getHeading()).getDegrees();
    }

    public boolean isReadyToFeed() { return readyToFeed; }
    public boolean isFeedEnabled() { return feedEnabled; }
    public boolean isAimRequested() { return aimRequested; }
    public boolean isShootRequested() { return shootRequested; }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AutoAlign/TargetHeadingDeg", getTargetHeading().getDegrees());
        SmartDashboard.putNumber("AutoAlign/ActualHeadingDeg", drive.getHeading().getDegrees());
        SmartDashboard.putNumber("AutoAlign/ErrorDeg", getHeadingErrorDegrees());
        SmartDashboard.putBoolean("AutoAlign/AtHeading", driveAtHeading());
        SmartDashboard.putBoolean("SuperStructure/AimRequested", aimRequested);
        SmartDashboard.putBoolean("SuperStructure/ShootRequested", shootRequested);
        SmartDashboard.putBoolean("SuperStructure/ReadyToFeed", readyToFeed);
        SmartDashboard.putBoolean("SuperStructure/FeedEnabled", feedEnabled);
    }
}
