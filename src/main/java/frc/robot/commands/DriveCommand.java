package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Joysticks;
import frc.robot.Robot;
import frc.robot.Joysticks.AlignMode;
import frc.robot.lib.math.MathHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.object.ObjectVision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommand extends Command {
    private final Drive drive;
    private final Supplier<Joysticks.DriveInputs> driveInputs;
    private final FuelTracking fuelTracking;
	private Command pathCommand = null;
    private boolean isTraking = false;
	private boolean isAligning = false;

	private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.5);
	private final SlewRateLimiter rLimiter = new SlewRateLimiter(4.5);

    private final SwerveRequest.FieldCentric driveNoHeading =
            new SwerveRequest.FieldCentric()
                    .withDeadband(Constants.Drive.MAX_SPEED * 0.05)
                    .withRotationalDeadband(
                            4.2
                                    * 0.05)
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
		// Logger.recordOutput("HUB_Distance", this.drive.getPose().getTranslation().getDistance(Constants.Field.HUB_CENTER));
        Joysticks.DriveInputs inputs = this.driveInputs.get();
		if (Robot.isRedAlliance.get()) inputs = inputs.getRedFlipped();

		if (!inputs.isRotateZero() && !RobotState.isAutonomous()) {
			if (this.pathCommand != null) {
				this.pathCommand.end(true);
				this.pathCommand = null;
			}
			this.isAligning = false;
			inputs.alignMode = AlignMode.None;
		}
		// inputs.alignMode = AlignMode.None;

		if (inputs.alignMode == AlignMode.PointAlign) {
			if (inputs.isNonZero())
				return;

			if (this.isAligning) {
				this.drive.stopModules();
				return;
			}

			if (this.pathCommand == null) {
				// Generate path
				Pose2d scorePose = this.drive.getClosestScorePoint();
				Pose2d approachPose = new Pose2d(
						4.9, 7.6, Rotation2d.kCCW_90deg);

				this.pathCommand = Commands.sequence(
						AutoBuilder.pathfindToPose(
								approachPose, Constants.Drive.CONSTRAINTS, 1.5),
						AutoBuilder.pathfindToPose(
								scorePose, Constants.Drive.CONSTRAINTS, 0.0));
				this.pathCommand.initialize();
			}

			if (!this.pathCommand.isFinished())
				this.pathCommand.execute();

			if (this.pathCommand.isFinished()) {
				this.pathCommand.end(true);
				this.pathCommand = null;
				
				this.isAligning = true;
				this.drive.stopModules();
			}
		} else if (inputs.alignMode == AlignMode.None) {
			this.isAligning = false;

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
			if (this.isAligning) {
				this.drive.stopModules();
				return;
			}

			Rotation2d targetAngle = MathHelpers.getAngleFromHub(this.drive.getPose());
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

		r = r * r;
		rot = rot * rot * Math.signum(rot);

		double xSpeed = r * Math.cos(theta) * Constants.Drive.MAX_VELOCITY;
		double ySpeed = r * Math.sin(theta) * Constants.Drive.MAX_VELOCITY;
		double rSpeed = rot * Constants.Drive.MAX_ANGULAR_VELOCITY;

		return new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
	}

	public double deadZone(double input, double deadZone) {
		if (Math.abs(input) < deadZone) return 0.0;
		else if (input > 1.0) return 1.0;
		else if (input < -1.0) return -1.0;
		else return (input - deadZone) / (1.0 - deadZone);
	}
}
