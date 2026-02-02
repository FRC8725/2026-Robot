// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.drive.Drive;

public class Robot extends LoggedRobot {
	private static final double loopOverrunWarningTimeout = 0.2;
	private Command autonomousCommand;
	private final RobotContainer robotContainer;

	public static final Supplier<Boolean> isRedAlliance =
			() -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
	public static final Supplier<Boolean> isInAllianceZone =
			() -> MathHelpers.mirrorIfRed(Drive.getInstance().getPose()).getX()
					< Constants.Field.ALLIANCE_LINE_X;

	public Robot() {
		super(Constants.ROBOT_PERIODIC);

		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

		if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            if (!DriverStation.isFMSAttached()) {
                Logger.addDataReceiver(new NT4Publisher());
            }
        } else if (Constants.RobotMode.isReplay) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else if (RobotBase.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        }

        Logger.start();
        if (!Logger.hasReplaySource()) {
            RobotController.setTimeSource(RobotController::getFPGATime);
        }

        SignalLogger.enableAutoLogging(false);
		this.robotContainer = new RobotContainer();

		try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

        DriverStation.silenceJoystickConnectionWarning(true);
        RobotController.setBrownoutVoltage(6.0);

		if (RobotBase.isSimulation()) {
			Drive.getInstance().resetOdometry(
					new Pose2d(2.0, 2.0, Rotation2d.kZero));					
		}
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = robotContainer.getAutonomousCommand();

		if (this.autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(this.autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (this.autonomousCommand != null) {
			this.autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}
}
