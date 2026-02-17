package frc.robot.commands;

import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Joysticks.AlignMode;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.State;
import frc.robot.subsystems.drive.Drive;

public class AutoRunnerCmd extends Command {
	private final SuperStructure superStructure;
	private final Drive drive;
	private final Timer timer = new Timer();
	private final Timer waitTimer = new Timer();
	private final List<Event> events;
	private final Trajectory<SwerveSample> trajectory;
	private Event currentWaitEvent = null;
	private Pose2d lastPose = null;
	private Command pathCommand = null;
	private int eventI = 0;
	public static boolean isAlignFinished = false;
	private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);

	public AutoRunnerCmd(
			SuperStructure superStructure, Drive drive, Trajectory<SwerveSample> trajectory) {
		this.superStructure = superStructure;
		this.drive = drive;
		this.trajectory = trajectory;
		this.driveWithHeading.HeadingController = Constants.Drive.FACING_HUB_PID;
		this.addRequirements(this.superStructure, this.drive);

		this.events = this.trajectory.events().stream()
				.sorted(Comparator.comparingDouble(e -> e.timestamp))
				.map(this::eventFromEventMarker)
				.collect(Collectors.toList());
	}

	public class Event {
		public final String name;
		public final double timestamp;
		public final SuperStructure.StructureInput inputs;
		public final Supplier<Boolean> waitCondition;
		public final AlignMode alignMode;

		public Event copyWithTimestamp(double newTimestamp) {
			return new Event(this.name, newTimestamp, this.inputs, this.waitCondition, this.alignMode);
		}

		public Event(
				String name, SuperStructure.StructureInput inputs) {
            this(name, 0.0, inputs, null, AlignMode.None);
        }

        public Event(
				String name, SuperStructure.StructureInput inputs,
				Supplier<Boolean> waitCondition, AlignMode alignMode) {
        	this(name, 0.0, inputs, waitCondition, alignMode);
    	}

        public Event(
				String name, double timestamp,
				SuperStructure.StructureInput inputs,
				Supplier<Boolean> waitCondition, AlignMode alignMode) {
            this.name = name;
            this.timestamp = timestamp;
            this.inputs = inputs;
            this.waitCondition = waitCondition;
            this.alignMode = alignMode;
    	}
	}

	private final List<Event> eventTypes = List.of(
		new Event(
			"wantIntake",
			new SuperStructure.StructureInput() {{ wantIntake = true; }}),
		new Event(
			"stopIntake",
			new SuperStructure.StructureInput() {{ wantIntake = false; }}),
		new Event(
			"zeroIntake",
			new SuperStructure.StructureInput() {{ zeroIntake = true; }}),
		new Event(
			"slideIntake",
			new SuperStructure.StructureInput() {{ slideIntake = true; }}),
		new Event(
			"zoneAlign",
			new SuperStructure.StructureInput() {{ wantScore = true; }},
			() -> SuperStructure.getInstance().stateTime.hasElapsed(2.5) && SuperStructure.getInstance().state == State.Shoot,
			AlignMode.ZoneAlign),
		new Event(
			"zoneAlignMore",
			new SuperStructure.StructureInput() {{ wantScore = true; }},
			() -> SuperStructure.getInstance().stateTime.hasElapsed(3.5) && SuperStructure.getInstance().state == State.Shoot,
			AlignMode.ZoneAlign),
		new Event(
			"outpose",
			new SuperStructure.StructureInput() {{}},
			() -> this.waitTimer.hasElapsed(2.0),
			AlignMode.None),
		new Event(
			"trackFuel",
			new SuperStructure.StructureInput() {{ wantTrack = true; }},
			() -> false,
			AlignMode.None),
		new Event(
			"pointAlign",
			new SuperStructure.StructureInput() {{ wantScore = true; }},
			() -> true,
			AlignMode.PointAlign));

	private Event eventFromEventMarker(EventMarker eventMarker) {
		for (Event type : this.eventTypes) {
			if (type.name.equals(eventMarker.event)) 
				return type.copyWithTimestamp(eventMarker.timestamp);
		}
		throw new Error("Unrecognized event mark D:");
	}

	private boolean shouldRunEvent(Event ev) {
		return this.timer.hasElapsed(ev.timestamp) || this.timer.hasElapsed(this.trajectory.getTotalTime());
	}
	
	@Override
	public void initialize() {
		this.timer.start();
		Pose2d initPose = this.trajectory.getInitialPose(Robot.isRedAlliance.get()).get();

		if (this.drive.getPose().getTranslation().getDistance(initPose.getTranslation())
				> Constants.Drive.STRATING_TOLERANCE)
			this.drive.resetOdometry(initPose);
	}

	@Override
	public void execute() {
		if (this.timer.isRunning()) 
			assert this.currentWaitEvent == null;
		else 
			assert this.currentWaitEvent != null;

		if (this.currentWaitEvent != null) {
			if (this.currentWaitEvent.alignMode != AlignMode.None) {
				this.runAlignment(this.currentWaitEvent.alignMode);
			} else {
				if (this.lastPose != null)
					this.drive.followPose(this.lastPose);
				else
					this.drive.stopModules();
			}
				
			if (this.currentWaitEvent.alignMode == AlignMode.PointAlign) {
				isAlignFinished = this.pathCommand == null;
			} else if (this.currentWaitEvent.alignMode == AlignMode.ZoneAlign) {
				Rotation2d targetAngle = MathHelpers.getAngleFromHub(this.drive.getPose());
				isAlignFinished = this.drive.withinTolerance(targetAngle);
			}  else {
				isAlignFinished = true;
			}

			if (this.currentWaitEvent.waitCondition != null
					&& this.currentWaitEvent.waitCondition.get()
					&& isAlignFinished) {
				this.stopAlignment();
				this.superStructure.emptyInputs();
				this.currentWaitEvent = null;

				this.waitTimer.stop();
				this.timer.start();
			}

			this.logOuputs();
			return;
		}

		if (this.currentWaitEvent == null
				&& this.eventI < events.size()
				&& this.shouldRunEvent(this.events.get(this.eventI))) {
			Event ev = this.events.get(eventI++);
			this.superStructure.inputs = ev.inputs;

			if (ev.waitCondition != null) {
				this.currentWaitEvent = ev;
				// this.waitingForAlign = ev.requireAlignment;
				this.waitTimer.restart();
				this.drive.stopModules();
				this.timer.stop();
			}
		}

		if (this.currentWaitEvent == null) {
			SwerveSample sample = this.trajectory.sampleAt(this.timer.get(), Robot.isRedAlliance.get())
					.orElse(this.trajectory.getFinalSample(Robot.isRedAlliance.get()).get());
			this.lastPose = sample.getPose();
			
			this.drive.followSample(sample);
		}
		this.logOuputs();
	}

	@Override
	public
	void end(boolean interrupted) {
		this.stopAlignment();
		this.drive.stopModules();
		this.superStructure.emptyInputs();
	}

	@Override
	public boolean isFinished() {
		return this.timer.hasElapsed(this.trajectory.getTotalTime())
				&& this.currentWaitEvent == null 
				&& this.eventI >= this.events.size();
	}

	private void logOuputs() {
		Logger.recordOutput("AutoRunner/IsTimerRunning", this.timer.isRunning());
		Logger.recordOutput("AutoRunner/WaitTimer", this.waitTimer.get());
		Logger.recordOutput("AutoRunner/Time", this.timer.get());
		Logger.recordOutput("AutoRunner/TrajTotalTime", this.trajectory.getTotalTime());
		Logger.recordOutput("AutoRunner/CurrentWaitEvent", this.currentWaitEvent == null ? "NULL" : this.currentWaitEvent.name);
		Logger.recordOutput("AutoRunner/isAlignFinished", isAlignFinished);
	}

	private void runAlignment(AlignMode mode) {
		if (this.pathCommand == null) {
			if (mode == AlignMode.PointAlign) {
				// Generate path
				Pose2d[] pathPoses = this.drive.getClosestScorePoints();
				
				this.pathCommand = Commands.sequence(
						AutoBuilder.pathfindToPose(
								pathPoses[0], Constants.Drive.CONSTRAINTS, 1.5),
						AutoBuilder.pathfindToPose(
								pathPoses[1], Constants.Drive.CONSTRAINTS, 1.5),
						AutoBuilder.pathfindToPose(
								pathPoses[2], Constants.Drive.CONSTRAINTS, 0.0));
			} else if (mode == AlignMode.ZoneAlign) {
				Rotation2d targetAngle = MathHelpers.getAngleFromHub(this.drive.getPose());
				this.drive.setControl(
						this.driveWithHeading
								.withVelocityX(0.0)
								.withVelocityY(0.0)
								.withTargetDirection(targetAngle));				
			}

			if (this.pathCommand != null)
				this.pathCommand.initialize();
		}

		if (this.pathCommand != null) {
			this.pathCommand.execute();

			if (this.pathCommand.isFinished()) {
                this.pathCommand.end(false);
				this.pathCommand = null;
                this.drive.stopModules();
            }
		}		
	}

	private void stopAlignment() {
		if (this.pathCommand != null) {
			this.pathCommand.end(true);
			this.pathCommand = null;
		}
		this.drive.stopModules();
	}
}
