# AdvantageScope mechanism validation

The hood encoder is zeroed in software at robot startup. **Place the hood physically at its
zero-angle hard reference before powering or rebooting the robot.** `Shooter/HoodZeroAssumed`
stays true as an explicit reminder that startup zeroing is an assumption, not a measured limit.

## Live simulator view

1. Start desktop simulation and enable teleop in the simulated Driver Station.
2. In AdvantageScope, choose **File -> Connect to Simulator -> NetworkTables 4**.
3. Add a Table tab with the requested-state fields, all `*At*` readiness booleans,
   `SuperStructure/ReadyToFeed`, `SuperStructure/FeedEnabled`, and the feeder,
   spindexer, and intake applied-output fields.
4. Add a Line Graph tab with intake pivot and roller setpoint/actual pairs, shooter
   flywheel and hood setpoint/actual pairs, and `AutoAlign/ErrorDeg`.

The intake's controlled-hardware tuning entry is `Intake/RollerActiveTargetRPM`; it starts
at 1800 roller-output RPM. `Intake/RollerSetpointRPM` reports the currently commanded speed,
including zero when the roller is off.

## Post-run log

Simulation starts WPILib data and Driver Station logging under `logs/generated/` in the
robot project. Open the generated `.wpilog` through **File -> Open Log(s)...** and reuse the
same Table and Line Graph layouts. Generated logs are ignored by Git.
