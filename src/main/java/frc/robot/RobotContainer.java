// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.center.HopperCenterIOHardware;
import frc.robot.subsystems.hopper.roller.HopperRollerIOHardware;

public class RobotContainer {
	private final Hopper hopper = this.buildHopperSubsystem();

	public Hopper buildHopperSubsystem() {
		return new Hopper(
				new HopperRollerIOHardware(),
				new	HopperCenterIOHardware());
	}
	
	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
