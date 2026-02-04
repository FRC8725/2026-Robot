package frc.robot.commands;

import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;

public class AutoRunnerCmd extends Command {
	private final SuperStructure superStructure;
	private final Drive drive;
	private final Timer timer = new Timer();
	private final List<Event> events;
	private SuperStructure.StructureInput postAlignInputs = null;
	private final Trajectory<SwerveSample> trajectory;
	private Event currentWaitEvent = null;
	private boolean waitingForAlign = false;
	private Pose2d lastPose = null;
	private Command pathCommand = null;
	private int eventI = 0;

	public AutoRunnerCmd(
			SuperStructure superStructure, Drive drive, Trajectory<SwerveSample> trajectory) {
		this.superStructure = superStructure;
		this.drive = drive;
		this.trajectory = trajectory;
		this.addRequirements(this.superStructure);

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

	// TODO
	private final List<Event> eventTypes = List.of(
		new Event(
			"wantIntake",
			new SuperStructure.StructureInput() {{ wantIntake = true; }}),
		new Event(
			"stopIntake",
			new SuperStructure.StructureInput() {{ wantIntake = false; }}),
		new Event(
			"zoneAlign",
			new SuperStructure.StructureInput() {{ wantScore = true; }},
			() -> (SuperStructure.getInstance().state == SuperStructure.State.Shoot), // TODO
			AlignMode.ZoneAlign),
		new Event(
			"trackFuel",
			new SuperStructure.StructureInput() {{ wantTrack = true; }},
			() -> false,
			AlignMode.None),
		new Event(
			"pointAlign",
			new SuperStructure.StructureInput() {{ wantScore = true; }},
			() -> true, // TODO
			AlignMode.PointAlign));

	private Event eventFromEventMarker(EventMarker eventMarker) {
		for (Event type : this.eventTypes) {
			// if (type.requireAlignment) assert type.waitCondition != null;
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
				
			if (this.currentWaitEvent.waitCondition != null && this.currentWaitEvent.waitCondition.get()
					&& (this.pathCommand == null || this.pathCommand.isFinished())) {
				this.stopAlignment();
				this.superStructure.emptyInputs();
				this.currentWaitEvent = null;

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
			// if (ev.) 
			// 	this.postAlignInputs = ev.inputs;
			// else 
			// 	this.superStructure.inputs = ev.inputs;

			if (ev.waitCondition != null) {
				this.currentWaitEvent = ev;
				// this.waitingForAlign = ev.requireAlignment;
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
		Logger.recordOutput("AutoRunner/Time", this.timer.get());
		Logger.recordOutput("AutoRunner/TrajTotalTime", this.trajectory.getTotalTime());
		Logger.recordOutput("AutoRunner/CurrentWaitEvent", this.currentWaitEvent == null ? "NULL" : this.currentWaitEvent.name);
	}

	private void runAlignment(AlignMode mode) {
		if (this.pathCommand == null) {
			if (mode == AlignMode.PointAlign) {
				// Generate path
				Pose2d scorePose = this.drive.getClosestScorePoint();

				Pose2d approachPose = new Pose2d(
						4.9, 7.6, Rotation2d.kCCW_90deg);

				this.pathCommand = Commands.sequence(
						AutoBuilder.pathfindToPose(
									approachPose, Constants.Drive.CONSTRAINTS, 1.5),
						AutoBuilder.pathfindToPose(
								scorePose, Constants.Drive.CONSTRAINTS, 0.0));
			} else if (mode == AlignMode.ZoneAlign) {
				this.pathCommand = AutoBuilder.pathfindToPose(
						Constants.Field.LEFT_POINT, Constants.Drive.CONSTRAINTS);
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
