package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.LifterIOHardware;
import frc.robot.subsystems.intake.LifterIOSim;
import frc.robot.subsystems.rollers.RollerIOHardware;
import frc.robot.subsystems.rollers.RollerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
	private final Consumer<VisionFieldPoseEstimate> visionEstimatorConsumer = 
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate poseEstimate) {
                    driveSubsystem.addVisionMeasurement(poseEstimate);
                }
            };

	private final RobotState robotState = new RobotState(this.visionEstimatorConsumer);
	private final DriveSubsystem driveSubsystem = this.buildDriveSubsystem();
	private final Intake intake = this.buildIntakeSubsystem();
	private final VisionSubsystem vision = this.buildVisionSubsystem();

	public DriveSubsystem buildDriveSubsystem() {
		if (RobotBase.isSimulation()) {
			return new DriveSubsystem(
					new DriveIOSim(
							this.robotState,
							Constants.Drive.drivetrain.getConstants(),
							Constants.Drive.drivetrain.getModuleConstants()));
		} else {
			return new DriveSubsystem(
					new DriveIOHardware(
							Constants.Drive.drivetrain.getConstants(),
							Constants.Drive.drivetrain.getModuleConstants()));
		}
	}

	// public Shooter buildShooterSubsystem() {
	// 	if (RobotBase.isSimulation()) {
	// 		return new Shooter(
	// 				new , null, null, null, null)
	// 	}
	// }

	public Intake buildIntakeSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Intake(
					new LifterIOSim(0),
					new RollerIOSim(
							DCMotor.getFalcon500(1), 1.0, 0.025));
		} else {
			return new Intake(
					new LifterIOHardware(0),
					new RollerIOHardware(0, false));
		}
	}

	public VisionSubsystem buildVisionSubsystem() {
		if (RobotBase.isSimulation()) {
			return new VisionSubsystem(
					new VisionIOSim(this.robotState),
					this.robotState);
		} else {
			return new VisionSubsystem(
					new VisionIOHardware(), this.robotState);
		}
	}

	public DriveSubsystem getDriveSubsystem() {
		return this.driveSubsystem;
	}

	private final XboxController controller = new XboxController(0);
	public RobotContainer() {
		this.driveSubsystem.setDefaultCommand(
				new DriveCommand(this.driveSubsystem, 
					this.controller::getLeftY, this.controller::getLeftX, this.controller::getRightX));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
