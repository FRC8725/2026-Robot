package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Joysticks;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.HoodState;

public class SuperStructureCmd extends Command {
	private final Joysticks joysticks;
	
	public SuperStructureCmd(Joysticks joysticks) {
		this.joysticks = joysticks;

		this.addRequirements(SuperStructure.getInstance());
	}

	@Override
	public void execute() {
		SuperStructure.getInstance().inputs = this.joysticks.getInput();
	}

	@Override
	public void end(boolean interrupted) {
		SuperStructure.getInstance().emptyInputs();
		if (this.joysticks.getInput().resetHood) {
			Shooter.getInstance().setHoodState(HoodState.Default);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
