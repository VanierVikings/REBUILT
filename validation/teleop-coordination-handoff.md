# Teleop coordination handoff

Last updated: 2026-09-16

## Resume point

- Upstream repository: `VanierVikings/REBUILT`
- Fork: `UwUltimatenub/REBUILT`
- Working branch: `codex/teleop-trigger-sim-validation`
- Latest implementation commit: `f0ec4e1`
- Pull request: <https://github.com/VanierVikings/REBUILT/pull/1>
- Pull request base: `commands-and-bs`
- Shooter orientation: robot-left, represented by a WPILib `+90 degree` yaw.

The coordinated intake, shooter, spindexer, feeder, and auto-align implementation is complete and
has passed deterministic simulation. Do not treat that result as proof of physical correctness.

## Completed validation

- `gradlew compileJava`: passed.
- `gradlew test --tests frc.robot.TeleopCoordinationSimulationTest`: passed.
- `gradlew test`: passed.
- Bounded `gradlew simulateJava -PheadlessSim` startup: passed; stopped intentionally after the
  robot, NT4, and WPILOG startup paths were observed.
- Left trigger: shooter spin-up, hood aiming, and drivetrain alignment reached their tolerances;
  feeder and spindexer stayed off.
- Right trigger: feed stayed off before readiness and enabled after the 100 ms readiness debounce.
- Both triggers: the right-trigger shooting command took priority.
- Trigger release: feeder and spindexer stopped within one 20 ms loop and the hood setpoint
  returned to zero.
- Left bumper: intake deployed and ran on the first press, then homed and stopped on the second.
- A deployed intake remained active while aiming and shooting.
- Blue and red alliance heading calculations passed with the left-facing shooter transform.
- All documented SmartDashboard fields existed in the simulator.
- Limelight covariance remained finite and positive across accepted angular velocities.

Fresh focused-test log:

- File: `logs/generated/FRC_20260917_005100.wpilog` (ignored by Git)
- SHA-256: `71f9ac90671283a382df60e61edf76b88d8d737bdaa9f46c345e3f8f24603431`
- Test report: `build/test-results/test/TEST-frc.robot.TeleopCoordinationSimulationTest.xml`
- Result: 1 test, 0 failures, 0 errors.

See `validation/teleop-coordination.md` for the complete validation record and
`docs/ADVANTAGESCOPE.md` for the telemetry layout.

## Work still required

1. Review pull request #1 with a human mentor/programmer before merging or deploying.
2. Open the simulator in AdvantageScope and perform the documented visual check:
   - confirm requested states and readiness booleans change with each binding;
   - graph flywheel and hood setpoint versus measurement;
   - graph heading error;
   - confirm feeder and spindexer outputs remain zero until readiness is stable for 100 ms;
   - open the generated WPILOG for post-run inspection.
3. Investigate the approximately 34 ms startup-only loop-overrun warning. Confirm it does not
   persist after startup on the target roboRIO.
4. Conduct controlled hardware checks with the robot safely supported and mechanisms clear:
   - physically place the hood at zero before boot and confirm `Shooter/HoodZeroAssumed`;
   - verify every motor inversion and sensor polarity before closed-loop motion;
   - verify CAN IDs, wiring, current limits, neutral modes, and software limits;
   - confirm the intake moves toward home/deployed setpoints rather than away from them;
   - confirm the hood moves toward its commanded angle;
   - confirm positive flywheel, feeder, spindexer, and intake commands rotate in the intended
     physical directions;
   - confirm disabled mode produces zero actuator output.
5. Repeat the driver-control acceptance sequence on hardware at reduced energy:
   - left bumper deploys/runs the intake and the next press homes/stops it;
   - left trigger aims and spins up without feeding;
   - right trigger does not feed early, then feeds when flywheel, hood, and heading are ready;
   - holding both triggers gives right-trigger priority without repeated interruption;
   - releasing triggers stops feeder/spindexer promptly and returns the hood to zero;
   - the intake remains safe while deployed during aiming and shooting.
6. Tune on hardware only after direction and safety checks:
   - intake roller target starts at 1,800 output RPM and is adjustable through
     `Intake/RollerActiveTargetRPM`;
   - validate the 3:1 intake conversion against the actual mechanism;
   - tune intake feedforward, flywheel/hood control, and final shooter RPM using logged data;
   - confirm the readiness tolerances of 2 RPS, 2 degrees, and 3 degrees are appropriate.
7. Validate blue- and red-side HUB alignment from multiple legal shooting poses. Confirm that the
   robot-left shooter faces the HUB and that drivetrain rotation has the correct sign.
8. Re-run the commands below after any code or tuning change and append the new revision, inputs,
   measurements, pass/fail results, log filename, and SHA-256 to the validation record.

## Resume commands

Run from `CodeBase/9659Code` with the WPILib 2026 JDK selected:

```powershell
git switch codex/teleop-trigger-sim-validation
.\gradlew.bat compileJava
.\gradlew.bat test --tests frc.robot.TeleopCoordinationSimulationTest --rerun-tasks
.\gradlew.bat test
$env:HALSIM_EXTENSIONS = ''
.\gradlew.bat simulateJava -PheadlessSim --console=plain
```

Generated logs belong under `logs/generated/` and must remain uncommitted. The local
`simgui-ds.json` modification is simulator-generated state and was intentionally excluded from the
implementation commit.

## Safety boundary

Simulation cannot validate real inversion, sensor polarity, hood startup position, CAN wiring,
mechanism loads, traction, camera accuracy, or final PID/RPM tuning. Do not deploy or enable this
AI-authored change on a robot without explicit authorization, human review, and controlled testing.
