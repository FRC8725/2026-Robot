package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.SuperStructureCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
	private final Joysticks joysticks = new Joysticks();
	private final Swerve swerve = new Swerve();
	private final Shooter shooter = new Shooter();
	private final Vision vision = new Vision();
	private final SuperStructure superStructure = new SuperStructure();

	public RobotContainer() {
	}

	public void teleInit() {
		this.swerve.setDefaultCommand(
				new DriveCmd(this.swerve, this.vision, this.joysticks::getDriveInput));
		this.superStructure.setDefaultCommand(new SuperStructureCmd(this.joysticks));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
