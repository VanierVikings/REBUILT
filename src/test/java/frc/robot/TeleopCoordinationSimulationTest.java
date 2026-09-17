package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.AlectronaLib.EstimatePose;
import frc.robot.subsystems.SuperStructure.IntakePivotStates;
import frc.robot.subsystems.SuperStructure.IntakeRollerStates;
import frc.robot.subsystems.SuperStructure.ShooterStates;
import frc.robot.subsystems.SuperStructure.SpindexerStates;
import frc.robot.subsystems.shooter.shotCalculator;

class TeleopCoordinationSimulationTest {
  private static Robot robot;
  private static RobotContainer container;
  private static XboxControllerSim driver;
  private static Thread robotThread;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    robot = new Robot();
    container = robot.getRobotContainerForTest();
    driver = new XboxControllerSim(Constants.DriveConstants.DriverPort);
    robotThread = new Thread(robot::startCompetition, "robot-test-loop");
    robotThread.setDaemon(true);
    robotThread.start();
    step(3);
  }

  @AfterAll
  static void tearDown() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step(2);
    CommandScheduler.getInstance().cancelAll();
    robot.endCompetition();
    try {
      robotThread.join(2000);
    } catch (InterruptedException exception) {
      Thread.currentThread().interrupt();
    }
    DataLogManager.stop();
    SimHooks.resumeTiming();
    robot.close();
  }

  @Test
  void coordinatedTeleopScenarios() {
    // Disabled startup.
    assertEquals(0.0, container.getShooterForTest().getFeederVoltage(), 1e-9);
    assertEquals(0.0, container.getSpindexerForTest().getAppliedVoltage(), 1e-9);
    assertEquals(0.0, container.getIntakeForTest().getRollerAppliedOutput(), 1e-9);

    enableTeleop();

    // Left bumper toggles the whole intake and the model reaches both positions.
    pressLeftBumper();
    step(70);
    assertEquals(IntakePivotStates.PIVOT_DEPLOYED, container.getIntakeForTest().getState());
    assertEquals(IntakeRollerStates.ROLLER_ACTIVE, container.getIntakeForTest().getRollerState());
    assertEquals(Constants.IntakeConstants.deployedAngle,
        container.getIntakeForTest().getIntakePivotAngle(), 3.0);
    assertTrue(container.getIntakeForTest().getRollerRPM() > 1000);

    pressLeftBumper();
    step(70);
    assertEquals(IntakePivotStates.PIVOT_HOME, container.getIntakeForTest().getState());
    assertEquals(IntakeRollerStates.ROLLER_OFF, container.getIntakeForTest().getRollerState());
    assertEquals(Constants.IntakeConstants.homeAngle,
        container.getIntakeForTest().getIntakePivotAngle(), 3.0);
    assertEquals(0.0, container.getIntakeForTest().getRollerRPM(), 1.0);

    // Leave intake deployed while aiming and shooting (no interlock).
    pressLeftBumper();
    step(70);
    aimRobotAtCurrentTarget();

    driver.setLeftTriggerAxis(1.0);
    DriverStationSim.notifyNewData();
    step(120);
    assertEquals(ShooterStates.AIMING, container.getShooterForTest().getState());
    assertTrue(container.getShooterForTest().shooterAtSpeed(2.0));
    assertTrue(container.getShooterForTest().hoodAtAngle(2.0));
    assertTrue(container.getSuperStructureForTest().driveAtHeading(),
        "heading error=" + container.getSuperStructureForTest().getHeadingErrorDegrees()
            + ", actual=" + container.getDrivetrainForTest().getHeading().getDegrees());
    assertEquals(0.0, container.getShooterForTest().getFeederVoltage(), 1e-9);
    assertEquals(0.0, container.getSpindexerForTest().getAppliedVoltage(), 1e-9);
    assertTrue(container.getIntakeForTest().isDeployed());

    // Right trigger preempts left trigger and feeding remains gated before the debounce.
    driver.setRightTriggerAxis(1.0);
    DriverStationSim.notifyNewData();
    step(2);
    assertTrue(container.getSuperStructureForTest().isShootRequested());
    assertFalse(container.getSuperStructureForTest().isAimRequested());
    assertEquals(0.0, container.getShooterForTest().getFeederVoltage(), 1e-9);
    assertEquals(SpindexerStates.OFF, container.getSpindexerForTest().getRequestedState());

    step(20);
    assertTrue(container.getSuperStructureForTest().isReadyToFeed(),
        "flywheel=" + container.getShooterForTest().shooterAtSpeed(2.0)
            + ", hood=" + container.getShooterForTest().hoodAtAngle(2.0)
            + ", headingError=" + container.getSuperStructureForTest().getHeadingErrorDegrees());
    assertTrue(container.getSuperStructureForTest().isFeedEnabled());
    assertTrue(container.getShooterForTest().getFeederVoltage() > 0);
    assertEquals(SpindexerStates.FEED, container.getSpindexerForTest().getRequestedState());
    assertTrue(container.getIntakeForTest().isDeployed());

    driver.setRightTriggerAxis(0.0);
    driver.setLeftTriggerAxis(0.0);
    DriverStationSim.notifyNewData();
    step(1);
    assertEquals(0.0, container.getShooterForTest().getFeederVoltage(), 1e-9);
    assertEquals(0.0, container.getSpindexerForTest().getAppliedVoltage(), 1e-9);
    assertEquals(0.0, SmartDashboard.getNumber("Shooter/HoodSetpointDeg", -1), 1e-9);

    verifyAllianceHeading(AllianceStationID.Blue1, new Pose2d(2.0, 3.0, new Rotation2d()));
    verifyAllianceHeading(AllianceStationID.Red1, new Pose2d(14.6, 5.0, new Rotation2d()));

    for (double omega : new double[] {0, 10, 25, 49.999}) {
      double multiplier = EstimatePose.rotationUncertaintyMultiplier(omega);
      assertTrue(Double.isFinite(multiplier) && multiplier > 0);
      assertEquals(1.0, multiplier, 1e-12);
    }

    step(2);
    NetworkTableInstance.getDefault().flush();
    List<String> keys = List.of(
        "Intake/RequestedPivotState", "Intake/PivotSetpointDeg", "Intake/PivotActualDeg",
        "Intake/PivotAtSetpoint", "Intake/RollerSetpointRPM", "Intake/RollerActualRPM",
        "Intake/RollerAppliedOutput", "Shooter/RequestedState", "Shooter/FlywheelSetpointRPS",
        "Shooter/FlywheelActualRPS", "Shooter/FlywheelErrorRPS", "Shooter/FlywheelAtSpeed",
        "Shooter/HoodSetpointDeg", "Shooter/HoodActualDeg", "Shooter/HoodAtAngle",
        "Shooter/FeederVoltage", "Shooter/HoodZeroAssumed", "Spindexer/RequestedState",
        "Spindexer/AppliedVoltage", "AutoAlign/TargetHeadingDeg", "AutoAlign/ActualHeadingDeg",
        "AutoAlign/ErrorDeg", "AutoAlign/AtHeading", "SuperStructure/AimRequested",
        "SuperStructure/ShootRequested", "SuperStructure/ReadyToFeed",
        "SuperStructure/FeedEnabled");
    for (String key : keys) {
      assertTrue(NetworkTableInstance.getDefault().getTable("SmartDashboard").containsKey(key), key);
    }
    assertEquals(container.getShooterForTest().getFeederVoltage(),
        SmartDashboard.getNumber("Shooter/FeederVoltage", Double.NaN), 1e-9);
  }

  private static void enableTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(2);
  }

  private static void pressLeftBumper() {
    driver.setLeftBumperButton(true);
    DriverStationSim.notifyNewData();
    step(1);
    driver.setLeftBumperButton(false);
    DriverStationSim.notifyNewData();
    step(1);
  }

  private static void aimRobotAtCurrentTarget() {
    for (int iteration = 0; iteration < 8; iteration++) {
      shotCalculator.getInstance().clearShootingParameters();
      Rotation2d heading = Rotation2d.fromRadians(
          shotCalculator.getInstance().getParameters().robotHeadingRadians());
      container.getDrivetrainForTest().resetOdometry(
          new Pose2d(container.getDrivetrainForTest().getPose().getTranslation(), heading));
    }
    shotCalculator.getInstance().clearShootingParameters();
  }

  private static void verifyAllianceHeading(AllianceStationID station, Pose2d pose) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
    container.getDrivetrainForTest().resetOdometry(pose);
    shotCalculator.getInstance().clearShootingParameters();
    var params = shotCalculator.getInstance().getParameters();
    Pose2d target = station == AllianceStationID.Red1
        ? Constants.fieldPoses.redAllianceHub : Constants.fieldPoses.blueAllianceHub;
    Pose2d shooterPose = pose.transformBy(new Transform2d(
        Constants.robotToShooter.getTranslation().toTranslation2d(),
        Constants.robotToShooter.getRotation().toRotation2d()));
    Rotation2d expected = target.getTranslation().minus(shooterPose.getTranslation()).getAngle()
        .minus(Constants.robotToShooter.getRotation().toRotation2d());
    assertEquals(0.0, expected.minus(Rotation2d.fromRadians(params.robotHeadingRadians())).getDegrees(), 1e-6);
  }

  private static void step(int loops) {
    for (int i = 0; i < loops; i++) {
      SimHooks.stepTiming(0.020);
    }
  }
}
