package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Joysticks;
import frc.robot.Robot;
import frc.robot.Joysticks.AlignMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Vision;

import java.util.function.Supplier;

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
                    .withDeadband(3.0 * 0.05)
                    .withRotationalDeadband(
                            8.2
                                    * 0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(0.05)
                    .withDriveRequestType(DriveRequestType.Velocity);

    public DriveCommand(
            Drive driveSubsystem,
			Vision vision,
            Supplier<Joysticks.DriveInputs> driveInputs) {
        this.drive = driveSubsystem;
		this.driveInputs = driveInputs;
        this.addRequirements(this.drive);
		this.fuelTracking = new FuelTracking(driveSubsystem, vision);

        this.driveWithHeading.HeadingController.setPID(12.0, 0.0, 0.0);

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

		if (inputs.isNonZero() && !RobotState.isAutonomous()) {
			if (this.pathCommand != null) {
				this.pathCommand.end(true);
				this.pathCommand = null;
			}
			this.isAligning = false;
			inputs.alignMode = AlignMode.None;
		}

		if (inputs.alignMode == AlignMode.PointAlign) {
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
				this.setSpeeds();
			}
		} else {

		}
		 
		if (this.isTraking) {
			this.fuelTracking.end(true);
			this.isTraking = false;
		}
		// Facing hub
		// Rotation2d targetAngle = MathHelpers.getAngleFromHub(this.driveSubsystem.getPose());
		// this.driveSubsystem.setControl(
		// 		this.driveNoHeading
		// 				.withVelocityX(speeds.vxMetersPerSecond)
		// 				.withVelocityY(speeds.vyMetersPerSecond)
		// 				.withRotationalRate(speeds.omegaRadiansPerSecond));		
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

	public void setSpeeds() {
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

		this.drive.setControl(
						this.driveNoHeading
								.withVelocityX(xSpeed)
								.withVelocityY(ySpeed)
								.withRotationalRate(rSpeed));
	}

	public double deadZone(double input, double deadZone) {
		if (Math.abs(input) < deadZone) return 0.0;
		else if (input > 1.0) return 1.0;
		else if (input < -1.0) return -1.0;
		else return (input - deadZone) / (1.0 - deadZone);
	}
}
