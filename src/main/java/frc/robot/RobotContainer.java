package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
	private final DriveSubsystem driveSubsystem = this.buildDriveSubsystem();
	
	private final Consumer<VisionFieldPoseEstimate> visionEstimatorConsumer = 
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate poseEstimate) {
                    driveSubsystem.addVisionMeasurement(poseEstimate);
                }
            };

	private final RobotState robotState = new RobotState(this.visionEstimatorConsumer);
	private final VisionSubsystem vision = this.buildVisionSubsystem();

	public DriveSubsystem buildDriveSubsystem() {
		if (RobotBase.isSimulation()) {
			return new DriveSubsystem(
					new DriveIOSim(
							Constants.Drive.drivetrain.getConstants(),
							Constants.Drive.drivetrain.getModuleConstants()));
		} else {
			return new DriveSubsystem(
					new DriveIOHardware(
							Constants.Drive.drivetrain.getConstants(),
							Constants.Drive.drivetrain.getModuleConstants()));
		}
	}

	public VisionSubsystem buildVisionSubsystem() {
		return new VisionSubsystem(
				new VisionIOHardware(), null);
	}

	public RobotContainer() {

	}

	public Command getAutonomousCommand() {
		return null;
	}
}
