// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SuperStructureCmd;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.lifter.LifterIOHardware;
import frc.robot.subsystems.intake.roller.IntakeRollerHardware;

public class RobotContainer {
	private final Joysticks joysticks = new Joysticks();
	private final Intake intake = this.buildHopperSubsystem();
	private final SuperStructure superStructure = new SuperStructure();

	public Intake buildHopperSubsystem() {
		return new Intake(
				new LifterIOHardware(),
				new IntakeRollerHardware());
	}
	
	public RobotContainer() {
		this.superStructure.setDefaultCommand(new SuperStructureCmd(this.joysticks));
		configureBindings();
	}

	private void configureBindings() {
		
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
