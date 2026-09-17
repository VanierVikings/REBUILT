# Teleop coordination validation contract

- Revision at implementation start: `289571b98d3bf6f4b31fef5d8aca174ced9ef5b2`
- Runner: `./gradlew test --tests frc.robot.TeleopCoordinationSimulationTest`
- Phase: disabled startup followed by teleop, stepped at 20 ms with paused simulation timing
- Inputs: blue/red alliance station, Xbox left bumper and trigger axes, legal alliance-zone poses
- Generated evidence: `logs/generated/*.wpilog` and Gradle XML/HTML test reports

Checks cover disabled zero outputs; intake deploy/home toggle and roller behavior; aim without
feeding; readiness-gated shooting and one-loop release stop; simultaneous-trigger priority;
concurrent deployed intake; blue/red HUB-facing headings for a left-facing (+90 degree) shooter;
positive accepted Limelight covariance; and the complete SmartDashboard key contract.

Simulation does not establish physical motor inversion, sensor polarity, CAN wiring, mechanism
loads, traction, camera accuracy, hood startup position, or final PID/RPM tuning. Those require
human review and controlled hardware testing before deployment.

## 2026-09-16 result

- `gradlew compileJava`: PASS
- `gradlew test --tests frc.robot.TeleopCoordinationSimulationTest`: PASS
- `gradlew test`: PASS (1 test, all scenarios)
- `gradlew simulateJava -PheadlessSim`: robot startup and NT4/WPILOG startup observed for a
  bounded 10-second window; stopped intentionally. A startup-only 34.0 ms loop-overrun warning
  remains visible and should be watched on the target system.
- Successful scenario log: `logs/generated/FRC_20260917_004328.wpilog`, 306317 bytes,
  SHA-256 `a23355deca4db0daf4edae3827c9eb593a9bc9d3a7eb4ec18bb84e98063e325d`.
- Headless startup log: `logs/generated/FRC_TBD_ea7354953682e736.wpilog`, 305493 bytes,
  SHA-256 `ce696482e92baea956b1d429c18c33887531164f02bb869d186bd166e90619a0`.
- WPILOG catalog: valid version 256; successful scenario log has 226 entries and 15773
  records; headless log has 187 entries and 16332 records. All documented SmartDashboard fields
  and Driver Station joystick fields were present. Focused anomaly scan reported zero anomalies.
- AdvantageScope was not exposed to the automation session, so the live visual layout was not
  manually inspected. The same underlying NT fields were asserted in JUnit and cataloged from
  WPILOG; `docs/ADVANTAGESCOPE.md` documents the remaining human visual check.
