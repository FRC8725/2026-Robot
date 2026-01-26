package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;

public class RobotContainer {
	private final Consumer<VisionFieldPoseEstimate> visionEstimatorConsumer = 
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate poseEstimate) {
                    drive.addVisionMeasurement(poseEstimate);
                }
            };

	private final Joysticks joysticks = new Joysticks();
	private final RobotState robotState = new RobotState(this.visionEstimatorConsumer);
	private final SuperStructure superStructure = new SuperStructure();
	private final Drive drive = this.buildDriveSubsystem();
	private final Vision objectVision = new Vision();

	public Drive buildDriveSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Drive(
					new DriveIOSim(
							this.robotState,
							Constants.Drive.drivetrain.getConstants(),
							Constants.Drive.drivetrain.getModuleConstants()));
		} else {
			return new Drive(
					new DriveIOHardware(
							Constants.Drive.drivetrain.getConstants(),
							Constants.Drive.drivetrain.getModuleConstants()));
		}
	}
	
	public RobotContainer() {
		this.drive.setDefaultCommand(
				new DriveCommand(this.drive, this.objectVision, this.joysticks::getDriveInput));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
