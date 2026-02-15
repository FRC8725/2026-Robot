package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Joysticks;
import frc.robot.Robot;
import frc.robot.Joysticks.AlignMode;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.object.ObjectVision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class DriveCommand extends Command {
    private final Drive drive;
    private final Supplier<Joysticks.DriveInputs> driveInputs;
    private final FuelTracking fuelTracking;
	private Command pathCommand = null;
    private boolean isTraking = false;
	public static boolean isAlignFinished = false;

	private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter rLimiter = new SlewRateLimiter(4.5);

    private final SwerveRequest.FieldCentric driveNoHeading =
            new SwerveRequest.FieldCentric()
                    .withDeadband(Constants.Drive.MAX_SPEED * 0.05)
                    .withRotationalDeadband(
                            Constants.Drive.MAX_ANGULAR_RATE * 0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);

    public DriveCommand(
            Drive driveSubsystem,
			ObjectVision vision,
            Supplier<Joysticks.DriveInputs> driveInputs) {
        this.drive = driveSubsystem;
		this.driveInputs = driveInputs;
        this.addRequirements(this.drive);
		this.fuelTracking = new FuelTracking(driveSubsystem, vision);

        this.driveWithHeading.HeadingController.setPID(5.0, 0.0, 0.0);

        if (RobotBase.isSimulation()) {
            this.driveNoHeading.DriveRequestType = DriveRequestType.OpenLoopVoltage;
            this.driveWithHeading.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        }
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Joysticks.DriveInputs inputs = this.driveInputs.get();
		if (Robot.isRedAlliance.get()) inputs = inputs.getRedFlipped();

		if (!inputs.isRotateZero() && !RobotState.isAutonomous()) {
			if (this.pathCommand != null) {
				this.pathCommand.end(true);
				this.pathCommand = null;
			}
			isAlignFinished = false;
			inputs.alignMode = AlignMode.None;
		}

		if (inputs.alignMode == AlignMode.PointAlign) {
			if (inputs.isNonZero())
				return;

			if (isAlignFinished) {
				this.drive.stopModules();
				return;
			}

			if (this.pathCommand == null) {
				// Generate path
				Pose2d[] pathPoses = this.drive.getClosestScorePoints();

				Pose2d robotPose = this.drive.getPose();

				List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
						new Pose2d(robotPose.getTranslation(),
								MathHelpers.mirrorIfRed(robotPose.getRotation().minus(Rotation2d.k180deg))),
						new Pose2d(pathPoses[0].getTranslation(), MathHelpers.mirrorIfRed(Rotation2d.k180deg)),
						new Pose2d(pathPoses[1].getTranslation(), MathHelpers.mirrorIfRed(Rotation2d.k180deg)),
						pathPoses[2]);
				List<RotationTarget> rotationTargets = new ArrayList<>();
				rotationTargets.add(new RotationTarget(1.0, MathHelpers.mirrorIfRed(Rotation2d.kZero)));
				rotationTargets.add(new RotationTarget(2.0, MathHelpers.mirrorIfRed(Rotation2d.kZero)));
				PathPlannerPath path = new PathPlannerPath(
					waypoints,
					rotationTargets,
					Collections.emptyList(),
					Collections.emptyList(),
					Collections.emptyList(),
					Constants.Drive.CONSTRAINTS,
					new IdealStartingState(0.0, this.drive.getPose().getRotation()),
					new GoalEndState(0.0, MathHelpers.negativeRotation(pathPoses[2].getRotation().plus(Rotation2d.k180deg))),
					false);

				path.preventFlipping = true;

				this.pathCommand = AutoBuilder.followPath(path);
				this.pathCommand.initialize();
			}

			if (!this.pathCommand.isFinished())
				this.pathCommand.execute();

			if (this.pathCommand.isFinished()) {
				this.pathCommand.end(true);
				this.pathCommand = null;
				
				isAlignFinished = true;
				this.drive.stopModules();
			}
		} else if (inputs.alignMode == AlignMode.None) {
			isAlignFinished = false;

			if (this.pathCommand != null) {
           		this.pathCommand.end(true);
            	this.pathCommand = null;
        	}

			if (inputs.wantTrack) {
				if (!this.isTraking) {
					this.fuelTracking.initialize();
					this.isTraking = true;
				}
				this.fuelTracking.execute();
			} else {
				ChassisSpeeds speeds = this.getSpeeds();
				this.drive.setControl(
						this.driveNoHeading
								.withVelocityX(speeds.vxMetersPerSecond)
								.withVelocityY(speeds.vyMetersPerSecond)
								.withRotationalRate(speeds.omegaRadiansPerSecond));
			}
		} else if (inputs.alignMode == AlignMode.ZoneAlign && inputs.isRotateZero()) {
			Rotation2d targetAngle = MathHelpers.getAngleFromHub(this.drive.getPose());

			if (this.drive.withinTolerance(targetAngle)) 
				isAlignFinished = true;
			
			ChassisSpeeds speeds = this.getSpeeds();

			this.drive.setControl(
					this.driveWithHeading
							.withVelocityX(speeds.vxMetersPerSecond)
							.withVelocityY(speeds.vyMetersPerSecond)
							.withTargetDirection(targetAngle));
		}
		
		if (this.isTraking) {
			this.fuelTracking.end(true);
			this.isTraking = false;
		}	
    }

	@Override
	public void end(boolean interrupted) {
		if (this.pathCommand != null) {
			this.pathCommand.end(true);
			this.pathCommand = null;
		}
		this.drive.stopModules();
	}

    @Override
    public boolean isFinished() {
        return false;
    }

	public ChassisSpeeds getSpeeds() {
		double x = this.xLimiter.calculate(-this.driveInputs.get().leftY);
		double y = this.yLimiter.calculate(-this.driveInputs.get().leftX);
		double rot = this.rLimiter.calculate(-this.driveInputs.get().rightX);

		double theta = Math.atan2(y, x);
		double r = Math.hypot(x, y);
		
		r = this.deadZone(r, this.driveInputs.get().deadZone);
		rot = this.deadZone(rot, this.driveInputs.get().deadZone);

		double xSpeed = r * Math.cos(theta) * Constants.Drive.MAX_VELOCITY;
		double ySpeed = r * Math.sin(theta) * Constants.Drive.MAX_VELOCITY;
		double rSpeed = rot * Constants.Drive.MAX_ANGULAR_RATE;

		return new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
	}

	public double deadZone(double input, double deadZone) {
		if (Math.abs(input) < deadZone) return 0.0;
		else if (input > 1.0) return 1.0;
		else if (input < -1.0) return -1.0;
		else return (input - deadZone) / (1.0 - deadZone);
	}
}
