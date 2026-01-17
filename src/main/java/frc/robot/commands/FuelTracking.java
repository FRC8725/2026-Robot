package frc.robot.commands;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class FuelTracking extends Command {
	private final Swerve swerve;
	private final Vision vision;

	private final PathConstraints constraints = new PathConstraints(
			3.0,
			2.0,
			3 * Math.PI,
			2 * Math.PI);

	public FuelTracking(Swerve swerve, Vision vision) {
		this.swerve = swerve;
		this.vision = vision;
		this.addRequirements(this.swerve);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		List<Pose2d> sortedPath = this.sortPath();

		if (sortedPath.size() < 2 || sortedPath == null) return;

		List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(sortedPath);

		ChassisSpeeds robotSpeeds = this.swerve.getSpeeds();
		Pose2d finalPose = sortedPath.get(sortedPath.size() - 1);
		IdealStartingState startState = new IdealStartingState(
				Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond),
				sortedPath.get(0).getRotation());
		GoalEndState endState = new GoalEndState(
				2.0,
				finalPose.getRotation());

		PathPlannerPath path = new PathPlannerPath(
				waypoints,
				this.constraints,
				startState,
				endState);
		path.preventFlipping = true;

		AutoBuilder.followPath(path);
	}

	@Override
	public void end(boolean interrupted) {
		this.swerve.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public List<Pose2d> sortPath() {
		List<Pose2d> sortedPath = new ArrayList<>();
		Pose2d simulatePose = this.swerve.getPose();
		List<Translation2d> fules = this.vision.getFuelsEstimator();

		while (!fules.isEmpty()) {
			Translation2d bestFuel = fules.stream()
                .min(Comparator.comparingDouble(this.vision::caculateWeight))
                .orElse(null);

			if (bestFuel != null) {
				Rotation2d newHeading = new Rotation2d(
						bestFuel.getX() - simulatePose.getX(),
						bestFuel.getY() - simulatePose.getY());
				sortedPath.add(new Pose2d(bestFuel, newHeading));

				simulatePose = new Pose2d(bestFuel, newHeading);
				fules.remove(bestFuel);
			}
		}

		return sortedPath;
	}
}
