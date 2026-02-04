// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.hopper.center.HopperCenterIOHardware;
import frc.robot.subsystems.hopper.roller.HopperRollerIOHardware;

public class RobotContainer {
	private final XboxController controller = new XboxController(0);
	private final Hopper hopper = this.buildHopperSubsystem();

	public Hopper buildHopperSubsystem() {
		return new Hopper(
				new HopperRollerIOHardware(),
				new	HopperCenterIOHardware());
	}
	
	public RobotContainer() {
		configureBindings();
		new Trigger(this.controller::getAButton)
				.onTrue(Commands.run(() -> this.hopper.hopperState = HopperState.Convey));
		new Trigger(this.controller::getAButton)
				.onFalse(Commands.run(() -> this.hopper.hopperState = HopperState.Off));
	}

	private void configureBindings() {
		
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
