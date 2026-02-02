package frc.robot.commands;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.object.ObjectVision;

public class FuelTracking extends Command {
	private final Drive drive;
	private final ObjectVision vision;

	private Command currentRunningPath = null;

	public FuelTracking(Drive drive, ObjectVision vision) {
		this.drive = drive;
		this.vision = vision;
		this.addRequirements(this.drive);
	}

	@Override
	public void initialize() {
		this.currentRunningPath = null;
	}

	@Override
	public void execute() {
		boolean needNewPath = (this.currentRunningPath == null) || this.currentRunningPath.isFinished();

		if (needNewPath) {
			if (this.currentRunningPath != null) {
				this.currentRunningPath.end(false);
			}

			List<Pose2d> sortedPath = this.sortPath();

			if (sortedPath == null || sortedPath.size() < 2) {
				this.currentRunningPath = null;
				return;
			}

			List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(sortedPath);
			ChassisSpeeds robotSpeeds = this.drive.getRobotChassisSpeeds();
			Pose2d finalPose = sortedPath.get(sortedPath.size() - 1);
			IdealStartingState startState = new IdealStartingState(
					Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond),
					sortedPath.get(0).getRotation());
			GoalEndState endState = new GoalEndState(
					2.0,
					finalPose.getRotation());

			PathPlannerPath path = new PathPlannerPath(
					waypoints,
					Constants.Drive.CONSTRAINTS,
					startState,
					endState);
			path.preventFlipping = true;

			currentRunningPath = AutoBuilder.followPath(path);
        	currentRunningPath.initialize();

			Logger.recordOutput("Auto/Fuel Tracking", path.getPathPoses().toArray(Pose2d[]::new));
		}
		
		if (this.currentRunningPath != null) {
			this.currentRunningPath.execute();
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (this.currentRunningPath != null) {
			this.currentRunningPath.end(interrupted);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public List<Pose2d> sortPath() {
		List<Pose2d> sortedPath = new ArrayList<>();
		Pose2d currentSimPose = this.drive.getPose();
		sortedPath.add(currentSimPose);

		List<Translation2d> fules = new ArrayList<>(this.vision.getFuelsEstimator());

		while (!fules.isEmpty()) {
			final Pose2d searchStartPose = currentSimPose;

			Translation2d bestFuel = fules.stream()
                	.min(Comparator.comparingDouble(
							f -> this.vision.caculateWeight(f, searchStartPose)))
					.filter(
							f -> this.vision.caculateWeight(f, searchStartPose) < Double.MAX_VALUE)
                	.orElse(null);

			if (bestFuel != null) {
				Rotation2d approachAngle = new Rotation2d(
						bestFuel.getX() - searchStartPose.getX(),
						bestFuel.getY() - searchStartPose.getY());

				Translation2d stopPoint = bestFuel.minus(
						new Translation2d(0.33, approachAngle));

				Pose2d targetPose = new Pose2d(stopPoint, approachAngle);
				sortedPath.add(targetPose);

				currentSimPose = targetPose;
				fules.remove(bestFuel);
			} else {
				break;
			}
		}

		if (sortedPath.size() < 2) return null;
		Logger.recordOutput("Auto/Sort Point", sortedPath.toArray(Pose2d[]::new));
		return sortedPath;
	}
}
