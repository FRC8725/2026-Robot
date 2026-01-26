package frc.robot.commands;

import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;

public class AutoRunnerCmd extends Command {
	private SuperStructure superStructure;
	private SuperStructure.StructureInput postAlignInputs = null;
	private Trajectory<SwerveSample> trajectory;
	private Event currentWaitEvent = null;
	private boolean waitingForAlign = false;
	private Pose2d lastPose = null;
	private int eventI = 0;
	private final Drive drive;
	private final Timer timer = new Timer();
	private final List<Event> events;

	public AutoRunnerCmd(
			Drive drive, SuperStructure superStructure, Trajectory<SwerveSample> trajectory) {
		this.drive = drive;
		this.superStructure = superStructure;
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
		public final boolean requireAlignment;

		public Event copyWithTimestamp(double newTimestamp) {
			return new Event(this.name, newTimestamp, this.inputs, this.waitCondition, this.requireAlignment);
		}

		public Event(String name, SuperStructure.StructureInput inputs) {
            this(name, 0.0, inputs, null, false);
        }

        public Event(String name, SuperStructure.StructureInput inputs, Supplier<Boolean> waitCondition, boolean requireAlignment) {
        	this(name, 0.0, inputs, waitCondition, requireAlignment);
    	}

        public Event(String name, double timestamp, SuperStructure.StructureInput inputs, Supplier<Boolean> waitCondition, boolean requireAlignment) {
            this.name = name;
            this.timestamp = timestamp;
            this.inputs = inputs;
            this.waitCondition = waitCondition;
            this.requireAlignment = requireAlignment;
    	}
	}

	private final List<Event> eventTypes = List.of(
		new Event(
			"startL4",
			new SuperStructure.StructureInput() {{
				
			}}));

	private Event eventFromEventMarker(EventMarker eventMarker) {
		for (Event type : this.eventTypes) {
			if (type.requireAlignment) assert type.waitCondition != null;
			if (type.name.equals(eventMarker.event)) {
				return type.copyWithTimestamp(eventMarker.timestamp);
			}
		}
		throw new Error("Unrecognized event mark D:");
	}

	private boolean shouldRunEvent(Event ev) {
		return this.timer.hasElapsed(ev.timestamp) || this.timer.hasElapsed(this.trajectory.getTotalTime());
	}

	@Override
	public void initialize() {
		this.timer.reset();
		Pose2d initPose = this.trajectory.getInitialPose(Robot.isRedAlliance.get()).get();
		if (this.drive.getPose().getTranslation().getDistance(initPose.getTranslation())
				> Constants.Drive.STRATING_TOLERANCE) {
			this.drive.resetOdometry(initPose);
		}
	}

	@Override 
	public void execute() {
		if (this.timer.isRunning()) assert this.currentWaitEvent == null;
		else assert this.currentWaitEvent != null;

		if (this.waitingForAlign) {
			assert this.lastPose != null;
			if (this.drive.withinTolerance(Objects.requireNonNull(this.lastPose).getTranslation())) {
				this.superStructure.input = this.postAlignInputs;
				this.waitingForAlign = false;
			}
			this.drive.followPose(Objects.requireNonNull(this.lastPose));
			return;
		}

		if (this.currentWaitEvent != null && this.currentWaitEvent.waitCondition != null && this.currentWaitEvent.waitCondition.get()) {
			this.superStructure.emptyInputs();
			this.currentWaitEvent = null;
		} else if (this.currentWaitEvent == null && this.eventI < events.size() && this.shouldRunEvent(this.events.get(this.eventI))) {
			Event ev = this.events.get(eventI++);
			if (ev.requireAlignment) {
				this.postAlignInputs = ev.inputs;
			} else {
				this.superStructure.input = ev.inputs;
			}

			if (ev.waitCondition != null) {
				this.currentWaitEvent = ev;
				this.waitingForAlign = ev.requireAlignment;
				this.drive.stopModules();
				this.timer.stop();
			}
		}

		if (this.currentWaitEvent == null) {
			this.timer.start();
			SwerveSample sample = this.trajectory.sampleAt(this.timer.get(), Robot.isRedAlliance.get())
				.orElse(this.trajectory.getFinalSample(Robot.isRedAlliance.get()).get());
			this.lastPose = sample.getPose();

			this.drive.followSample(sample);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.superStructure.emptyInputs();
	}

	@Override
	public boolean isFinished() {
		return this.timer.hasElapsed(this.trajectory.getTotalTime()) &&
			this.currentWaitEvent == null &&
			this.eventI >= this.events.size();
	}
}
