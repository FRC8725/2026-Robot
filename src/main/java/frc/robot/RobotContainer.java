package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SuperStructureCmd;
import frc.robot.lib.limelight.VisionFieldPoseEstimate;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.center.HopperCenterIOHardware;
import frc.robot.subsystems.hopper.center.HopperCenterIOSim;
import frc.robot.subsystems.hopper.roller.HopperRollerIOHardware;
import frc.robot.subsystems.hopper.roller.HopperRollerIOSim;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.lifter.LifterIOHardware;
import frc.robot.subsystems.intake.lifter.LifterIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerHardware;
import frc.robot.subsystems.intake.roller.IntakeRollerSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.feeder.FeederIOHardware;
import frc.robot.subsystems.shooter.feeder.FeederIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOHardware;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOHardware;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
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
	private final Intake intake = this.buildIntakeSubsystem();
	private final Shooter shooter = this.buildShooterSubsystem();
	private final Hopper hopper = this.buildHopperSubsystem();
	private final Vision vision = this.buildVisionSubsystem();
	private final ObjectVision objectVision = new ObjectVision();
	private final Simulation simulation = this.buildSimulation();
	private final Leds leds = new Leds();

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
					new FlywheelIOSim(),
					new HoodIOSim(),
					new FeederIOSim(),
					this.joysticks.up,
					this.joysticks.down,
					this.joysticks.fup,
					this.joysticks.fdown);
		} else {
			return new Shooter(
					new FlywheelIOHardware(),
					new HoodIOHardware(), 
					new FeederIOHardware(),
					this.joysticks.up,
					this.joysticks.down,
					this.joysticks.fup,
					this.joysticks.fdown);
		}
	}

	public Intake buildIntakeSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Intake(
					new LifterIOSim(),
					new IntakeRollerSim());
		} else {
			return new Intake(
					new LifterIOHardware(),
					new IntakeRollerHardware());
		}
	}

	public Hopper buildHopperSubsystem() {
		if (RobotBase.isSimulation()) {
			return new Hopper(
					new HopperRollerIOSim(),
					new HopperCenterIOSim());
		} else {
			return new Hopper(
					new HopperRollerIOHardware(),
					new HopperCenterIOHardware());
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

	public Simulation buildSimulation() {
		if (RobotBase.isSimulation()) {
			return new Simulation(this);
		} else {
			return null;
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
