// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOHardware;
import frc.robot.subsystems.shooter.hood.HoodIOHardware;

public class RobotContainer {
	private final XboxController controller = new XboxController(0);
	private final Shooter shooter = this.buildShooterSubsystem();

	public Shooter buildShooterSubsystem() {
		return new Shooter(
				new FlywheelIOHardware(0),
				new HoodIOHardware(0));
	}

	private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(
					null, // Use default ramp rate (1 V/s)
					Volts.of(8), // Reduce dynamic step voltage to 4 to prevent brownout
					null, // Use default timeout (10 s)
							// Log state with Phoenix SignalLogger class
					(state) -> SignalLogger.writeString("state", state.toString())),
			new SysIdRoutine.Mechanism(
					(volts) -> this.shooter.setFlywheelVolts(volts.in(Volts)),
					null,
					this.shooter));

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

	public Command getTester() {
		return Commands.sequence(
				this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
						.withTimeout(10.0),
				new WaitCommand(1.5),
				this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
						.withTimeout(10.0),
				new WaitCommand(1.5),
				this.sysIdDynamic(SysIdRoutine.Direction.kForward)
						.withTimeout(3.0),
				new WaitCommand(1.5),
				this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
						.withTimeout(3.0));
	}

	public RobotContainer() {
		new Trigger(this.controller::getAButton)
				.whileTrue(this.getTester());
		new Trigger(this.controller::getLeftBumperButton)
				.onTrue(Commands.runOnce(SignalLogger::start));
		new Trigger(this.controller::getRightBumperButton)
				.onTrue(Commands.runOnce(SignalLogger::stop));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
