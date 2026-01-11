// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Motor;

public class RobotContainer {
	private final XboxController controller = new XboxController(0);
	private final Motor motor = new Motor();

	public RobotContainer() {
		new Trigger(this.controller::getLeftBumperButton)
			.onTrue(Commands.runOnce(() -> Motor.VALUE += (this.motor.getMode() == "Voltage" ? 1.0 : 0.1)));
		new Trigger(this.controller::getRightBumperButton)
			.onTrue(Commands.runOnce(() -> Motor.VALUE -= (this.motor.getMode() == "Voltage" ? 1.0 : 0.1)));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
