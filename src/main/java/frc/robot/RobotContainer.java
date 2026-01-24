package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.lifter.LifterIOHardware;
import frc.robot.subsystems.intake.lifter.LifterIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerHardware;
import frc.robot.subsystems.intake.roller.IntakeRollerSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.feeder.FeederIOHardware;
import frc.robot.subsystems.shooter.feeder.FeederIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOHardware;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOHardware;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
	private final Consumer<VisionFieldPoseEstimate> visionEstimatorConsumer = 
            new Consumer<VisionFieldPoseEstimate>() {
                @Override
                public void accept(VisionFieldPoseEstimate poseEstimate) {
                    drive.addVisionMeasurement(poseEstimate);
                }
            };

	private final RobotState robotState = new RobotState(this.visionEstimatorConsumer);
	private final Drive drive = this.buildDriveSubsystem();
	private final Intake intake = this.buildIntakeSubsystem();
	private final Shooter shooter = this.buildShooterSubsystem();
	private final Vision vision = this.buildVisionSubsystem();

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

	public Shooter buildShooterSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Shooter(
					new FlywheelIOSim(0),
					new HoodIOSim(0),
					new FeederIOSim());
		} else {
			return new Shooter(
					new FlywheelIOHardware(0, 0),
					new HoodIOHardware(0), 
					new FeederIOHardware());
		}
	}

	public Intake buildIntakeSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Intake(
					new LifterIOSim(0),
					new IntakeRollerSim());
		} else {
			return new Intake(
					new LifterIOHardware(0),
					new IntakeRollerHardware());
		}
	}

	public Vision buildVisionSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Vision(
					new VisionIOSim(this.robotState),
					this.robotState);
		} else {
			return new Vision(
					new VisionIOHardware(), this.robotState);
		}
	}

	private final XboxController controller = new XboxController(0);
	public RobotContainer() {
		this.drive.setDefaultCommand(
				new DriveCommand(this.drive, 
					this.controller::getLeftY, this.controller::getLeftX, this.controller::getRightX));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
