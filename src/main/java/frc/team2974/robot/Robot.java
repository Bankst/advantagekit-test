// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2974.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.BehaviorConstants;
import frc.robot.BuildConstants;
import frc.team2974.robot.RobotConstants.Mode;
import frc.team2974.robot.subsystems.drive.*;

public class Robot extends LoggedRobot {
	private Command autonCommand;
	private double autonStartTime = 0.0;
	private boolean autonMessagePrinted = false;

	private final CommandXboxController driverXbox = new CommandXboxController(0);

	private DriveS drive;

	public Robot() {
		// Initialize subsystems
		if (RobotConstants.currentMode == Mode.REAL) {
			drive = new DriveS(
				new GyroIOPigeon2(),
				new ModuleIOCTRE(0),
				new ModuleIOCTRE(1),
				new ModuleIOCTRE(2),
				new ModuleIOCTRE(3)
			);
		} else if (RobotConstants.currentMode == Mode.SIM) {
			drive = new DriveS(
				new GyroIO() {},
				new ModuleIOSim(0),
				new ModuleIOSim(1),
				new ModuleIOSim(2),
				new ModuleIOSim(3)
			);
		}

		// Instantiate missing subsystems
		if (drive == null) {
      drive = new DriveS(
				new GyroIO() {},
				new ModuleIO() {},
				new ModuleIO() {},
				new ModuleIO() {},
				new ModuleIO() {}
			);
		}
	}

	private CommandBase getAutonCommand() {
		return Commands.none();
	}

	private void bindHumanInputs() {
		drive.setDefaultCommand(
			new TeleDriveC(drive,
				()-> driverXbox.getRawAxis(1),
				()-> driverXbox.getRawAxis(0),
				()-> -driverXbox.getRawAxis(2),
				()-> false)
		);
	}

	@Override
	public void robotInit() {
		Logger logger = Logger.getInstance();
		Logger.getInstance().recordMetadata("ProjectName", "Bubbles-AK"); // Set a metadata value

		// Record metadata
    logger.recordMetadata("RobotMode", RobotConstants.currentMode.toString());
    System.out.println("[Init] Starting AdvantageKit");
    logger.recordMetadata("TuningMode", Boolean.toString(BehaviorConstants.kTuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

		// Set up data receivers & replay source
    switch (RobotConstants.currentMode) {
      case REAL:
				logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        logger.addDataReceiver(new NT4Publisher());
				LoggedPowerDistribution.getInstance(0, ModuleType.kRev);
        break;

      case SIM:
        logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(path));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

		// Start AdvantageKit logger
    setUseTiming(RobotConstants.currentMode != Mode.REPLAY);
    logger.start();

		// Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
			String name = command.getName();
			int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
			commandCounts.put(name, count);
			Logger.getInstance().recordOutput(
				"CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
			Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
		};
    CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
			logCommandFunction.accept(command, true);
		});
    CommandScheduler.getInstance().onCommandFinish((Command command) -> {
			logCommandFunction.accept(command, false);
		});
    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
			logCommandFunction.accept(command, false);
		});

		// Default to blue alliance in sim
    if (RobotConstants.currentMode == Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

		// Set up human inputs
		System.out.println("[Robot][Init] Binding Human Inputs");
		bindHumanInputs();
	}

	@Override
	public void robotPeriodic() {
		Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();

		// Log list of NT clients
    List<String> clientNames = new ArrayList<>();
    List<String> clientAddresses = new ArrayList<>();
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      clientNames.add(client.remote_id);
      clientAddresses.add(client.remote_ip);
    }
    Logger.getInstance().recordOutput(
			"NTClients/Names", clientNames.toArray(new String[clientNames.size()]));
    Logger.getInstance().recordOutput(
			"NTClients/Addresses", clientAddresses.toArray(new String[clientAddresses.size()]));

		// Print auto duration
    if (autonCommand != null) {
      if (!autonCommand.isScheduled() && !autonMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.println(String.format(
						"*** Auto finished in %.2f secs ***", Timer.getFPGATimestamp() - autonStartTime));
        } else {
          System.out.println(String.format(
						"*** Auto cancelled in %.2f secs ***", Timer.getFPGATimestamp() - autonStartTime));
        }
        autonMessagePrinted = true;
      }
    }

		Threads.setCurrentThreadPriority(true, 10);
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		autonStartTime = Timer.getFPGATimestamp();
    autonMessagePrinted = false;
    autonCommand = getAutonCommand();
    if (autonCommand != null) {
      autonCommand.schedule();
    }
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (autonCommand != null) {
			autonCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
