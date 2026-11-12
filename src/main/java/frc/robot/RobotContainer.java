package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SuperStructureCmd;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.apriltag.VisionIOHardware;
import frc.robot.subsystems.vision.apriltag.VisionIOSim;
import frc.robot.subsystems.vision.object.ObjectVision;

public class RobotContainer {
	private final Consumer<VisionFieldPoseEstimate> visionEstimatorConsumer = 
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate poseEstimate) {
                    drive.addVisionMeasurement(poseEstimate);
                }
            };

	private final RobotState robotState = new RobotState(this.visionEstimatorConsumer);
	private final SuperStructure superStructure = new SuperStructure();
	private final Joysticks joysticks = new Joysticks();
	private final Drive drive = this.buildDriveSubsystem();
	private final Vision vision = this.buildVisionSubsystem();
	private final ObjectVision objectVision = new ObjectVision();

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

	public Vision buildVisionSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Vision(
					new VisionIOSim(this.robotState),
					this.robotState);
		} else {
			return new Vision(
					new VisionIOHardware(),
					this.robotState);
		}
	}

	public Drive getDriveSubsystem() {
		return this.drive;
	}

	public SuperStructure getSuperStructure() {
		return this.superStructure;
	}

	public RobotContainer() {
	}

	public void initializeTeleoperate() {
		this.drive.setDefaultCommand(
				new DriveCommand(this.drive, this.objectVision, this.joysticks::getDriveInput));
		this.superStructure.setDefaultCommand(new SuperStructureCmd(this.joysticks));
	}
}
